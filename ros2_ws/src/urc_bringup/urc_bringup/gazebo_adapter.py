"""Normalize Gazebo Harmonic native sensors into the URC wire contract."""
import base64
import io
import json
import math
from pathlib import Path
import re

import numpy as np
from PIL import Image as PillowImage
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, String
from urc_msgs.msg import GripperCommand
from urc_core.geometry import camera_mount, header, inv, pose_matrix, pose_msg, stamp, time_value, transform_point, vector
from urc_core.runtime import SENSORS, Twin, diagonal
from .node import to_ros


class GazeboAdapter(Node):
    def __init__(self):
        super().__init__('urc_gazebo_adapter')
        self.declare_parameter('manifest','')
        self.manifest=json.loads(Path(self.get_parameter('manifest').value).read_text())
        self.sequence=0; self.last_scan=None; self.pending=0; self.state=None; self.info=None; self.depth=None
        self.depth_time=-1.
        self.rng=np.random.default_rng(int(self.manifest['seed']))
        self.outputs={name:self.create_publisher(get_message(SENSORS[name]),name,10) for name in (
            '/drone/lidar/packet','/drone/range/down','/drone/scan','/drone/camera/image/compressed','/drone/camera/camera_info',
            '/drone/camera/depth/image_raw','/perception/oracle/detections_2d','/perception/oracle/depth_samples',
            '/localization/tag_observations','/drone/gripper/status','/bins/delivery_event','/joint_states','/sim/control/response')}
        self.cmd=self.create_publisher(Twist,'/gz/urc/cmd_vel',10)
        self.stop=self.create_publisher(Bool,'/gz/urc/estop',10)
        self.grip=self.create_publisher(String,'/gz/urc/gripper',10)
        self.create_subscription(TwistStamped,'/drone/cmd_vel',self.velocity,10)
        self.create_subscription(Bool,'/drone/estop',self.stop.publish,10)
        self.create_subscription(GripperCommand,'/drone/gripper/command',self.gripper,10)
        self.create_subscription(String,'/sim/control/request',self.control,10)
        self.create_subscription(String,'/slam/status',self.ack,10)
        self.create_subscription(String,'/gz/urc/state',self.read_state,10)
        self.create_subscription(PointCloud2,'/gz/urc/points',self.lidar,qos_profile_sensor_data)
        self.create_subscription(Image,'/gz/urc/image',self.image,qos_profile_sensor_data)
        self.create_subscription(Image,'/gz/urc/depth',self.read_depth,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'/gz/urc/camera_info',self.camera_info,qos_profile_sensor_data)

    def send(self,topic,payload):
        publisher=self.outputs[topic]
        publisher.publish(to_ros(publisher.msg_type,payload))

    def json(self,topic,content):
        self.send(topic,{'data':json.dumps(content,allow_nan=False)})

    def velocity(self,msg):
        ts=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        now=self.get_clock().now().nanoseconds*1e-9
        if -0.025<=now-ts<=.25:
            self.cmd.publish(msg.twist)

    def gripper(self,msg):
        if re.fullmatch('[a-zA-Z0-9_-]{1,64}',msg.request_id) and msg.command in ('open','close'):
            self.grip.publish(String(data=f'{msg.request_id} {msg.command}'))

    def control(self,msg):
        value=json.loads(msg.data)
        self.json('/sim/control/response',{'epoch':1,'request_id':value.get('request_id',''),'ok':False,
                  'reason':'Use Gazebo GUI pause/step. Regenerate the seeded SDF and relaunch to reset.'})

    def ack(self,msg):
        value=json.loads(msg.data)
        if value.get('ack_seq')==self.pending:
            self.pending=0

    def clearance(self,pose):
        x,y,z,_=pose;s=np.clip((x-2.6)/5.4,0,1);p=self.manifest['terrain_phase']
        elevation=s*s*(3-2*s)*self.manifest['terrain_amplitude']*(math.sin(.31*x+p)+.6*math.cos(.42*y-p)+.32*math.sin(.68*(x+y)))
        if abs(x)<1.1 and abs(y)<1.1: elevation=.019
        for mark in ('bin_a_map_xyz','bin_b_map_xyz'):
            bx,by,bz=self.manifest[mark]
            if abs(x-bx)<.6 and abs(y-by)<.6:elevation=max(elevation,bz+.06)
        for rock in self.manifest['rocks']:
            if math.dist((x,y),rock['xy'])<rock['radius_m']*.73:elevation=max(elevation,rock['height_m']*.96)
        return max(.03,z-.18-elevation)

    def read_state(self,msg):
        state=json.loads(msg.data);self.state=state; t=state['t']
        self.send('/joint_states',{'header':header(t,''),'name':['camera_tilt_joint'],'position':[math.pi/2]})
        self.send('/drone/range/down',{'header':header(t,'range_down_link'),'radiation_type':1,'min_range':.03,'max_range':20.,'range':self.clearance(state['pose'])})
        gripper={'header':header(t,'gripper_link'),**state['gripper'],'slip_detected':False,'reason':'Gazebo joint state'}
        self.send('/drone/gripper/status',gripper)
        for token,dest in state['events']:
            self.send('/bins/delivery_event',{'header':header(t,'map'),'payload_token':token,'bin_id':dest,'event_id':'1-'+token})

    @staticmethod
    def cloud(msg):
        offsets={field.name:field.offset for field in msg.fields if field.datatype==7}
        endian='>' if msg.is_bigendian else '<'
        if not set('xyz')<=offsets.keys(): return np.empty((0,3))
        data=np.frombuffer(bytes(msg.data),dtype=np.uint8).reshape(msg.height,msg.row_step)
        fields=[]
        for key in 'xyz':
            offset=offsets[key]
            arr=np.ndarray((msg.height,msg.width),dtype=endian+'f4',buffer=data,offset=offset,strides=(msg.row_step,msg.point_step))
            fields.append(arr.reshape(-1))
        return np.column_stack(fields)

    def lidar(self,msg):
        if self.state is None: return
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.pending and self.last_scan is not None and t-self.last_scan<.95:return
        cloud=self.cloud(msg);cloud=cloud[np.isfinite(cloud).all(axis=1)]
        if len(cloud)<60:return
        scan=np.full(360,20.)
        near=cloud[np.abs(cloud[:,2])<.38]
        indices=np.rint((np.arctan2(near[:,1],near[:,0])+math.pi)*180/math.pi).astype(int)%360
        np.minimum.at(scan,indices,np.linalg.norm(near[:,:2],axis=1))
        self.send('/drone/scan',{'header':header(t,'lidar_link'),'angle_min':-math.pi,'angle_max':math.pi-math.pi/180,
                  'angle_increment':math.pi/180,'range_min':.08,'range_max':20.,'ranges':scan.tolist()})
        dt=.1 if self.last_scan is None else t-self.last_scan
        if dt<=0:return
        self.sequence+=1;self.pending=self.sequence;self.last_scan=t
        source=self.state
        body=cloud[::max(1,len(cloud)//1200)].copy();body[:,2]+=.18
        self.json('/drone/lidar/packet',{'version':2,'epoch':1,'seq':self.sequence,'points':body.tolist(),
                  'stamp':stamp(t),'frame_id':'base_link','dt':dt,'gyro_z':source['gyro']+self.rng.normal(0,.001),
                  'flow_xy':(np.array(source['flow'])+self.rng.normal(0,.01,2)).tolist(), 'height':None,
                  'baro_height':source['pose'][2]-.65+self.rng.normal(0,.025),'clearance_down':self.clearance(source['pose'])})

    def camera_info(self,msg):
        self.info=message_to_ordereddict(msg)

    def image(self,msg):
        if msg.encoding not in ('rgb8','bgr8','rgba8'):return
        channels=4 if msg.encoding=='rgba8' else 3
        image=np.frombuffer(bytes(msg.data),dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*channels].reshape(msg.height,msg.width,channels)
        if msg.encoding=='bgr8': image=image[:,:,::-1]
        buffer=io.BytesIO();PillowImage.fromarray(image[:,:,:3]).save(buffer,format='JPEG',quality=80)
        capture={'sec':msg.header.stamp.sec,'nanosec':msg.header.stamp.nanosec};h={'stamp':capture,'frame_id':'camera_optical_frame'}
        self.send('/drone/camera/image/compressed',{'header':h,'format':'rgb8; jpeg compressed bgr8','data':base64.b64encode(buffer.getvalue()).decode()})

    def read_depth(self,msg):
        if msg.encoding not in ('32FC1','16UC1') or self.state is None or self.info is None:return
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if t-self.depth_time<.18:return
        self.depth_time=t;h=header(t,'camera_optical_frame');info={**self.info,'header':h}
        fmt=('>' if msg.is_bigendian else '<')+('f4' if msg.encoding=='32FC1' else 'u2')
        samples=np.frombuffer(bytes(msg.data),dtype=fmt).reshape(msg.height,-1)[:,:msg.width]
        metres=samples if msg.encoding=='32FC1' else samples*.001
        mm=np.where(np.isfinite(metres)&(metres>.035)&(metres<60),metres*1000.,0).astype('<u2')
        self.send('/drone/camera/camera_info',info)
        self.send('/drone/camera/depth/image_raw',{'header':h,'height':msg.height,'width':msg.width,'encoding':'16UC1',
                  'is_bigendian':0,'step':2*msg.width,'data':base64.b64encode(mm.tobytes()).decode()})
        self.synthetic_observations(t,mm,info)

    def synthetic_observations(self,t,mm,info):
        trans=inv(pose_matrix(self.state['pose'])@camera_mount(math.pi/2))
        h=info['header'];fx,cx,fy,cy=[info['k'][i] for i in (0,2,4,5)]
        boxes=[];depths=[];tags=[]
        def visible(p,thickness):
            if p[2]<=.12:return None
            x,y=int(round(fx*p[0]/p[2]+cx)),int(round(fy*p[1]/p[2]+cy))
            if not(0<=x<mm.shape[1] and 0<=y<mm.shape[0]):return None
            z=mm[y,x]*.001
            return (x,y,z) if z>0 and abs(p[2]-z)<thickness else None
        for kind,*xyz in self.state['targets']:
            p=transform_point(trans,np.asarray(xyz));hit=visible(p,.22)
            if hit is None or self.rng.random()<.15:continue
            x,y,z=hit; w=np.clip(fx*.16/z,3,125);height=np.clip(fy*.19/z,3,125)
            boxes.append({'header':h,'bbox':{'center':{'position':{'x':float(x),'y':float(y)},'theta':0.},
                          'size_x':float(w),'size_y':float(height)},'results':[{'hypothesis':{'class_id':kind,'score':.94}}]})
            depths.append([int(x),int(y),float(z)])
        from urc_core.geometry import pose_from_msg, quaternion
        for known in self.manifest['tag_poses_map']:
            fix=trans@pose_from_msg(known)
            if visible(fix[:3,3],.20) is None or np.linalg.norm(fix[:3,3])>12 or self.rng.random()<.10:continue
            tags.append({'tag_id':int(known['tag_id']),'confidence':.65,'pose':{'pose':{'position':vector(fix[:3,3]+self.rng.normal(0,.03,3)),
                         'orientation':quaternion(fix[:3,:3])},'covariance':diagonal([.001]*6)}})
        self.send('/perception/oracle/detections_2d',{'header':h,'detections':boxes})
        self.json('/perception/oracle/depth_samples',{'stamp':stamp(t),'epoch':1,'samples':depths})
        self.send('/localization/tag_observations',{'header':h,'observations':tags})


def main():
    rclpy.init();node=GazeboAdapter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
