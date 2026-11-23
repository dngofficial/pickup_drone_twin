"""Typed ROS 2 Jazzy adapter for the shared Twin runtime."""
import base64
import copy
import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CompressedImage
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
import yaml
from urc_msgs.action import StartMission
from urc_msgs.srv import SelectTarget, SetBinPolicy, SetDetector, SetGripper, SceneControl
from urc_core.runtime import SENSORS, OUTPUTS, Twin
from urc_core.geometry import header, quaternion, vector, yaw_rotation

LATCHED = {'/drone/estop','/slam/map','/mission/path','/mission/coverage','/drone/battery','/mission/status',
           '/objects/tracks','/localization/status','/visualization/objects','/visualization/site'}
FAST = {'/clock','/drone/scan','/drone/range/down','/drone/camera/image/compressed','/drone/camera/camera_info',
        '/drone/camera/depth/image_raw','/perception/oracle/detections_2d','/localization/tag_observations'}


def to_ros(msg_type, payload):
    data=copy.deepcopy(payload)
    if isinstance(data.get('data'),str) and msg_type.__module__.split('.')[0] in ('sensor_msgs',):
        # set_message_fields accepts bytes for uint8 arrays. Avoid materializing
        # hundreds of thousands of Python integers for each image or cloud.
        data['data']=base64.b64decode(data['data'],validate=True)
    result=msg_type()
    set_message_fields(result,data)
    return result


def from_ros(message):
    if isinstance(message,(Image,CompressedImage)):
        # The generic converter recursively visits every byte while holding the
        # controller lock, delaying LiDAR acknowledgments and flight commands.
        fields=message_to_ordereddict(message,no_arr=True)
        fields['data']=bytes(message.data)
        return fields
    return message_to_ordereddict(message)


class TwinNode(Node):
    def __init__(self):
        super().__init__('urc_twin')
        self.declare_parameter('config','')
        self.declare_parameter('detector','oracle')
        self.declare_parameter('model','')
        self.declare_parameter('autostart',False)
        cfg=yaml.safe_load(Path(self.get_parameter('config').value).read_text())
        self.lock=threading.RLock()
        self.callbacks=ReentrantCallbackGroup()
        # The runtime is serialized by its lock. Scheduling sensor callbacks
        # concurrently only queues lock waiters and delays fresh LiDAR/clock data.
        self.sensor_callbacks=MutuallyExclusiveCallbackGroup()
        self.inference_worker=ThreadPoolExecutor(max_workers=1,thread_name_prefix='urc_yolo')
        self.inference_busy=False
        self.by_topic={name:get_message(typ) for name,typ in OUTPUTS.items()}
        self.output_publishers={name:self.create_publisher(self.by_topic[name],name,
                         QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
                         if name in LATCHED else (qos_profile_sensor_data if name.endswith('/overlay/image') else 10)) for name in OUTPUTS}
        self.tf=TransformBroadcaster(self)
        self.core=Twin(cfg,self.send,self.get_parameter('detector').value,self.get_parameter('model').value or None,
                       autostart=self.get_parameter('autostart').value,inference_request=self.request_inference)
        for name,typ in SENSORS.items():
            self.create_subscription(get_message(typ),name,lambda msg,n=name:self.receive(n,msg),
                                     qos_profile_sensor_data if name in FAST else 10,callback_group=self.sensor_callbacks)
        for name,method in {'/mission/pause':'pause_mission','/mission/resume':'resume_mission',
                            '/safety/estop':'estop','/safety/clear_estop':'clear_estop'}.items():
            self.create_service(Trigger,name,lambda request,response,m=method:self.rpc(m,{},response),callback_group=self.callbacks)
        for typ,name,method,fields in [(SetBinPolicy,'/mission/set_bin_policy','set_bin_policy',('policy','selected_bin')),
                    (SelectTarget,'/mission/select_target','select_target',('track_id',)),
                    (SetDetector,'/perception/set_detector','set_detector',('mode',))]:
            self.create_service(typ,name,lambda req,res,m=method,f=fields:self.rpc(m,{key:getattr(req,key) for key in f},res),callback_group=self.callbacks)
        self.create_service(SetGripper,'/drone/gripper/set',self.gripper,callback_group=self.callbacks)
        self.create_service(SceneControl,'/sim/control',self.scene,callback_group=self.callbacks)
        self.action=ActionServer(self,StartMission,'/mission/start',self.execute,goal_callback=self.goal,
                                 cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.callbacks)

    def request_inference(self,stamp,jpeg):
        # Called under the controller lock. Keep one job in flight and let newer
        # paired frames replace skipped work rather than building a stale queue.
        if self.inference_busy:
            return False
        self.inference_busy=True
        epoch,detector=self.core.epoch,self.core.yolo
        future=self.inference_worker.submit(detector.infer,jpeg)
        future.add_done_callback(lambda result:self.finish_inference(stamp,epoch,detector,result))
        return True

    def finish_inference(self,stamp,epoch,detector,future):
        try:
            boxes=future.result()
        except Exception as exc:
            with self.lock:
                self.inference_busy=False
            self.get_logger().error(f'YOLO inference failed: {type(exc).__name__}',throttle_duration_sec=3.)
            return
        with self.lock:
            self.inference_busy=False
            if self.core.epoch==epoch and self.core.yolo is detector:
                self.core.complete_inference(stamp,boxes)

    def goal(self, req):
        with self.lock:
            mission=self.core.mission
            return GoalResponse.ACCEPT if req.mission_id and not mission.active and mission.outcome is None and not mission.estop else GoalResponse.REJECT

    def execute(self, goal):
        with self.lock:
            ready=self.core.mission.start(goal.request.mission_id)
        result=StartMission.Result()
        if not ready:
            result.reason='mission already active or interlocked'; goal.abort(); return result
        canceled=False
        while rclpy.ok():
            with self.lock:
                mission=self.core.mission
                if goal.is_cancel_requested and not canceled:
                    canceled=True;mission.request_return('canceled by action client');mission.paused=False
                feedback=StartMission.Feedback()
                feedback.status=to_ros(self.by_topic['/mission/status'],self.core.status())
                goal.publish_feedback(feedback)
                if not mission.active:
                    result.success=bool(mission.outcome and not canceled)
                    result.reason=mission.reason
                    result.delivered_count=len(mission.deliveries)
                    if canceled: goal.canceled()
                    elif result.success: goal.succeed()
                    else: goal.abort()
                    return result
            time.sleep(.2)
        goal.abort();return result

    def rpc(self, method, args, response):
        with self.lock:
            result=self.core.rpc(method,args)
        response.success=result['ok']
        response.message=result['result'] or result['error']
        return response

    def gripper(self,req,res):
        with self.lock:
            m=self.core.mission
            res.accepted=not m.active and not m.estop and bool(req.request_id) and req.command in ('open','close')
            if res.accepted:
                self.send('/drone/gripper/command',{'header':header(self.core.now,'gripper_link'),'request_id':req.request_id,'command':req.command})
            res.message='sent to actuator' if res.accepted else 'gripper interlocked or invalid command'
        return res

    def scene(self,req,res):
        lookup={'pause':'pause_sim','resume':'resume_sim','reset':'reset_scene','step':'step_sim'}
        res.epoch=self.core.epoch
        return self.rpc(lookup.get(req.operation,''),{'seed':req.seed,'ticks':req.ticks},res)

    def receive(self, topic, msg):
        with self.lock:
            self.core.on_message(topic,from_ros(msg))
            if topic=='/clock' and self.core.slam.valid and self.core.now-self.core.slam.latest<=.6:
                self.tf.sendTransform([self.transform('map','odom',self.core.fusion.applied),
                                       self.transform('odom','base_link',self.core.slam.pose)])

    def transform(self,parent,child,pose):
        value=TransformStamped()
        set_message_fields(value,{'header':header(self.core.now,parent),'child_frame_id':child,
                                 'transform':{'translation':vector(pose[:3]),'rotation':quaternion(yaw_rotation(pose[3]))}})
        return value

    def send(self,topic,payload):
        if topic in self.output_publishers:
            try:
                self.output_publishers[topic].publish(to_ros(self.by_topic[topic],payload))
            except (ValueError,TypeError,AttributeError,AssertionError) as exc:
                self.get_logger().error(f'{topic}: {exc}',throttle_duration_sec=3.)


def main():
    rclpy.init()
    node=TwinNode()
    # Keep a second worker for the blocking mission action's feedback loop.
    executor=MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown();node.inference_worker.shutdown(wait=True);node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
