"""Mission and perception processing for the ROS 2 node."""
import base64
from collections import deque
import io
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from .geometry import (camera_mount, header, matrix_pose, pose_from_msg, pose_matrix, pose_msg,
                       quaternion, stamp, time_value, transform_point, vector, wrap, yaw_rotation)
from .mission import Mission
from .onnx_detector import OnnxDetector
from .projection import backproject, foreground_point
from .slam import LidarSlam
from .station_fusion import StationFusion
from .tracker import Tracker

SENSORS = {
    '/clock':'rosgraph_msgs/msg/Clock', '/drone/lidar/packet':'std_msgs/msg/String',
    '/drone/range/down':'sensor_msgs/msg/Range', '/drone/scan':'sensor_msgs/msg/LaserScan',
    '/drone/camera/image/compressed':'sensor_msgs/msg/CompressedImage',
    '/drone/camera/camera_info':'sensor_msgs/msg/CameraInfo', '/drone/camera/depth/image_raw':'sensor_msgs/msg/Image',
    '/perception/oracle/detections_2d':'vision_msgs/msg/Detection2DArray',
    '/perception/oracle/depth_samples':'std_msgs/msg/String', '/localization/tag_observations':'urc_msgs/msg/TagObservationArray',
    '/drone/gripper/status':'urc_msgs/msg/GripperStatus', '/bins/delivery_event':'urc_msgs/msg/DeliveryEvent',
    '/joint_states':'sensor_msgs/msg/JointState', '/ui/request':'std_msgs/msg/String',
    '/sim/control/response':'std_msgs/msg/String', '/control/manual':'geometry_msgs/msg/TwistStamped',
}
OUTPUTS = {
    '/drone/cmd_vel':'geometry_msgs/msg/TwistStamped', '/drone/estop':'std_msgs/msg/Bool',
    '/drone/camera/tilt_cmd':'std_msgs/msg/Float64', '/drone/gripper/command':'urc_msgs/msg/GripperCommand',
    '/slam/status':'std_msgs/msg/String', '/slam/map':'sensor_msgs/msg/PointCloud2', '/ui/response':'std_msgs/msg/String',
    '/ui/telemetry':'std_msgs/msg/String', '/drone/odom':'nav_msgs/msg/Odometry',
    '/drone/pose':'geometry_msgs/msg/PoseWithCovarianceStamped', '/localization/status':'urc_msgs/msg/LocalizationStatus',
    '/perception/detections_2d':'vision_msgs/msg/Detection2DArray', '/perception/detections_3d':'vision_msgs/msg/Detection3DArray',
    '/objects/tracks':'urc_msgs/msg/TrackedObjectArray', '/cam_object_pose':'geometry_msgs/msg/PoseWithCovarianceStamped',
    '/drone/camera/overlay/image':'sensor_msgs/msg/Image', '/mission/status':'urc_msgs/msg/MissionStatus',
    '/mission/path':'nav_msgs/msg/Path', '/mission/coverage':'nav_msgs/msg/OccupancyGrid',
    '/drone/battery':'sensor_msgs/msg/BatteryState', '/visualization/objects':'visualization_msgs/msg/MarkerArray',
    '/visualization/site':'visualization_msgs/msg/MarkerArray', '/sim/control/request':'std_msgs/msg/String',
}
STATIC_TAGS = {
    10: ([-.45, 1.38, 1.8], [1,0,0], [0,0,1]),
    11: ([-.64, 1.19, 1.8], [0,-1,0], [0,0,1]),
    12: ([-.83, 1.38, 1.8], [-1,0,0], [0,0,1]),
    13: ([-.64, 1.57, 1.8], [0,1,0], [0,0,1]),
}


def diagonal(values):
    flat = np.zeros((6,6))
    flat[np.diag_indices(6)] = values
    return flat.ravel().tolist()


def as_bytes(data):
    return base64.b64decode(data,validate=True) if isinstance(data,str) else bytes(data)


class Twin:
    def __init__(self, config, send, detector='oracle', model=None, autostart=False, inference_request=None):
        self.config, self.send = config, send
        self.wanted_detector, self.model_path = detector, model
        self.yolo = None
        self.inference_request = inference_request
        self.tags = self.default_tags(config)
        self.autostart = autostart
        self.epoch = 1
        self.now = self.last_tick = 0.
        self.last_ui = self.last_map = -1.
        self.ui_responses = {}
        self.inbox = {}
        self.bad_input = 0
        self.reset(1)
        if detector == 'yolo':
            self.set_detector(detector)

    @staticmethod
    def default_tags(config):
        station = dict(STATIC_TAGS)
        for index, name in ((20,'bin_a'),(21,'bin_b')):
            x,y,z = config['scene'][name+'_map_xyz']
            station[index] = ([x-(config['scene']['bin_inner_width_m']+.16)*.5-.18,
                               y,z+config['scene']['bin_rim_height_m']+.025], [0,0,1], [1,0,0])
        transforms = {}
        for key,(point,normal,up) in station.items():
            # The stand's FLU transform has x opposite the graphical tag normal.
            x = -np.asarray(normal)
            z = np.asarray(up)
            matrix = np.eye(4)
            matrix[:3,:3] = np.column_stack([x,np.cross(z,x),z])
            matrix[:3,3] = point
            transforms[key] = matrix
        return transforms

    def reset(self, epoch):
        self.epoch, self.sequence = epoch, 0
        self.slam = LidarSlam(self.config['localization'])
        self.fusion = StationFusion(self.config['scene']['initial_base_map_xyz_yaw'], self.config['localization'])
        self.fusion.last_fix = self.now
        self.tracker = Tracker(self.config['perception'])
        self.mission = Mission(self.config)
        self.last_ui = self.last_map = self.now-1.
        self.raw_grip = {}
        self.clearance, self.range_time = math.nan, -1.
        self.history = deque(maxlen=65)
        self.tilt = math.radians(self.config['sensors']['search_camera_tilt_deg'])
        self.sent_tilt = None
        self.info, self.depth, self.boxes, self.sparse, self.jpegs = {}, {}, {}, {}, {}
        self.last_boxes = []
        self.last_project, self.last_image = -1., -1.
        self.manual_cmd = None
        self.last_camera_observation = -1.
        self.scan = []
        self.inbox.clear()
        self.session_started = self.now

    def set_detector(self, detector):
        if detector not in ('oracle','yolo'):
            raise ValueError('choose oracle or yolo')
        if self.mission.active and not self.mission.paused:
            raise ValueError('pause before changing detector')
        if detector=='yolo' and self.yolo is None:
            if not self.model_path:
                raise ValueError('set model to an exported YOLOv8 checkpoint')
            conf = self.config['perception']
            self.yolo = OnnxDetector(self.model_path,conf['input_size'],conf['confidence_threshold'],conf['nms_iou_threshold'])
        if detector != self.wanted_detector:
            # Retain picked/delivered tracks and identities while replacing observations.
            for track in self.tracker.tracks.values():
                track.seen = self.now - 10
            self.boxes.clear()
            self.sparse.clear()
        self.wanted_detector = detector

    def emit(self, topic, message):
        self.send(topic, message)

    def json(self, topic, value):
        self.emit(topic, {'data':json.dumps(value,separators=(',',':'),allow_nan=False)})

    @property
    def pose(self):
        return matrix_pose(pose_matrix(self.fusion.applied) @ pose_matrix(self.slam.pose))

    def on_message(self, topic, msg):
        if topic not in SENSORS:
            return
        try:
            self.process(topic,msg)
        except (ValueError, KeyError, TypeError, IndexError, OverflowError, np.linalg.LinAlgError, OSError):
            self.bad_input += 1

    def process(self, topic, msg):
        if topic == '/clock':
            now = time_value(msg['clock'])
            if now >= self.now:
                self.now = now
                if now-self.last_tick >= .049:
                    self.tick()
            return
        if topic == '/ui/request':
            request = json.loads(msg['data'])
            key = request['request_id']
            if key not in self.ui_responses:
                if len(self.ui_responses)>128:
                    self.ui_responses.pop(next(iter(self.ui_responses)))
                result = self.rpc(request['method'],request.get('args',{}),request.get('epoch'))
                self.ui_responses[key] = {'request_id':key,'epoch':self.epoch,**result}
            self.json('/ui/response',self.ui_responses[key])
            return
        if topic == '/sim/control/response':
            result = json.loads(msg['data'])
            if int(result['epoch']) > self.epoch:
                self.reset(int(result['epoch']))
            return
        if topic == '/perception/oracle/depth_samples':
            data = json.loads(msg['data'])
            t = round(time_value(data['stamp']),6)
            if self.wanted_detector == 'oracle' and data['epoch'] == self.epoch:
                self.sparse[t] = data['samples']
                self.perceive(t)
            return
        if topic == '/drone/lidar/packet':
            p = json.loads(msg['data'])
            if p['version'] != 2 or int(p['epoch']) < self.epoch:
                return
            if int(p['epoch']) > self.epoch:
                self.reset(int(p['epoch']))
            seq = int(p['seq'])
            t = time_value(p['stamp'])
            if seq > self.sequence:
                self.sequence = seq
                good = self.slam.update(p,t)
                if good:
                    self.history.append((t,self.slam.pose.copy()))
            self.json('/slam/status',{'epoch':self.epoch,'ack_seq':seq,'state':'TRACKING' if self.slam.registered else 'PREDICTING',
                                     'pose':self.slam.pose.tolist(),'rmse':self.slam.rmse,'inliers':self.slam.inliers,'map_points':len(self.slam.points)})
            if good and self.wanted_detector=='yolo':
                # RGB-D can arrive before its interpolated LiDAR pose. Retry the
                # newest usable pair after acknowledging the flight sensor.
                for image_time in sorted(self.depth,reverse=True):
                    self.perceive(image_time)
                    if self.last_project==image_time:
                        break
            return
        t = round(time_value(msg['header']['stamp']),6)
        if t < self.session_started-.02:
            return
        if topic == '/drone/range/down':
            self.clearance,self.range_time = float(msg['range']),t
        elif topic == '/drone/scan':
            self.scan = msg['ranges']
        elif topic == '/drone/gripper/status':
            self.raw_grip = msg
        elif topic == '/bins/delivery_event':
            if msg['bin_id'] in self.mission.bins:
                self.mission.deliver(msg,self.tracker.tracks)
        elif topic == '/joint_states' and 'camera_tilt_joint' in msg['name']:
            self.tilt = msg['position'][msg['name'].index('camera_tilt_joint')]
        elif topic == '/localization/tag_observations' and self.history:
            self.correct(msg,t)
        elif topic == '/control/manual':
            self.manual_cmd = (t,msg['twist'])
        elif topic == '/drone/camera/camera_info':
            self.info[t] = msg
            self.perceive(t)
        elif topic == '/drone/camera/depth/image_raw' and self.wanted_detector=='yolo':
            if msg['encoding']=='16UC1' and msg['step']==2*msg['width']:
                self.depth[t] = np.frombuffer(as_bytes(msg['data']),dtype='>u2' if msg['is_bigendian'] else '<u2').reshape(msg['height'],msg['width'])
                self.perceive(t)
        elif topic == '/perception/oracle/detections_2d' and self.wanted_detector=='oracle':
            boxes = []
            for box in msg['detections']:
                bound = box['bbox']; center=bound['center']['position']; kind=box['results'][0]['hypothesis']
                boxes.append((kind['class_id'],kind['score'],[center['x'],center['y'],bound['size_x'],bound['size_y']]))
            self.boxes[t] = boxes
            self.publish_boxes(t,boxes)
            self.perceive(t)
        elif topic == '/drone/camera/image/compressed':
            jpeg = as_bytes(msg['data'])
            self.jpegs[t] = jpeg
            if self.wanted_detector=='yolo':
                self.perceive(t)
            elif t in self.boxes:
                self.overlay(t)
        for cache in (self.info,self.depth,self.boxes,self.sparse,self.jpegs):
            while len(cache)>14:
                cache.pop(next(iter(cache)))

    def local_at(self,t):
        if not self.history:
            return None
        for (a,p),(b,q) in zip(self.history,list(self.history)[1:]):
            if a<=t<=b:
                part=(t-a)/max(b-a,1e-9)
                result=p+part*(q-p)
                result[3]=wrap(p[3]+part*wrap(q[3]-p[3]));return result
        closest=min(self.history,key=lambda row:abs(row[0]-t))
        if abs(closest[0]-t)<=.20:
            guess=closest[1].copy()
            guess[:3] += self.slam.velocity*(t-closest[0])
            return guess
        return None

    def correct(self,msg,t):
        pose = self.local_at(t)
        if pose is None:
            return
        known=[ob for ob in msg['observations'] if int(ob['tag_id']) in self.tags and ob['confidence']>=.35]
        if not known:
            return
        observed=max(known,key=lambda item:item['confidence'])
        measurement=pose_from_msg(observed['pose']['pose'])
        if not .1 < measurement[2,3] < 12.1:
            return
        fix=self.fusion.tag_fix(self.tags[int(observed['tag_id'])],measurement,pose,self.tilt)
        raw=np.asarray(observed['pose']['covariance']).reshape(6,6)
        sigmas=np.maximum(.001,np.diag(raw)[[0,1,2,5]]) + np.array([.06**2,.06**2,.05**2,.025**2])
        self.fusion.correct(fix,np.diag(sigmas),t,self.now,np.linalg.norm(self.slam.velocity)<.12)

    def publish_boxes(self,t,boxes):
        self.last_boxes = [(k,float(s),list(map(float,b))) for k,s,b in boxes]
        h=header(t,'camera_optical_frame')
        objects=[]
        for kind,score,(u,v,w,hgt) in boxes:
            objects.append({'header':h,'bbox':{'center':{'position':{'x':float(u),'y':float(v)},'theta':0.},'size_x':float(w),'size_y':float(hgt)},
                            'results':[{'hypothesis':{'class_id':kind,'score':float(score)}}],'id':''})
        self.emit('/perception/detections_2d',{'header':h,'detections':objects})

    def perceive(self,t):
        if t<=self.last_project or t not in self.info:
            return
        oracle=self.wanted_detector=='oracle'
        if oracle and (t not in self.boxes or t not in self.sparse):
            return
        if not oracle and (t not in self.depth or t not in self.jpegs):
            return
        local=self.local_at(t)
        if local is None:
            return
        if not oracle and t not in self.boxes:
            # Infer only complete RGB-D pairs at the depth rate. Unpaired RGB
            # frames otherwise occupy the sensor callback and delay LiDAR.
            if self.inference_request is not None:
                if t>self.last_image and self.inference_request(t,self.jpegs[t]):
                    self.last_image=t
                return
            self.boxes[t] = self.yolo.infer(self.jpegs[t])
            self.last_image=t
            self.publish_boxes(t,self.boxes[t])
            self.overlay(t)
        self.last_project=t
        self.last_camera_observation=t
        transform=pose_matrix(local)@camera_mount(self.tilt)
        correction=pose_matrix(self.fusion.applied)
        observations=[]; ros=[]
        for i,(kind,score,box) in enumerate(self.boxes[t]):
            if score < self.config['perception']['confidence_threshold'] or kind not in ('bottle','hammer'):
                continue
            sample = self.sparse[t][i] if oracle and i<len(self.sparse[t]) else (None if oracle else foreground_point(self.depth[t],box))
            if sample is None:
                continue
            p=backproject(*sample,self.info[t]['k'])
            point=transform_point(transform,p)
            if any(np.linalg.norm(transform_point(correction,point)[:2]-b[:2])<.85 for b in self.mission.bins.values()):
                continue
            observations.append((kind,score,point))
            xyz=transform_point(correction,point)
            observed=pose_msg([*xyz,0.])
            ros.append({'header':header(t,'map'),'results':[{'hypothesis':{'class_id':kind,'score':float(score)},'pose':{'pose':observed,'covariance':diagonal([.03,.03,.025,1000,1000,1000])}}],
                        'bbox':{'center':observed,'size':vector([.20,.28,.12])}})
        self.tracker.update(observations,t)
        self.emit('/perception/detections_3d',{'header':header(t,'map'),'detections':ros})

    def complete_inference(self,t,boxes):
        if (self.wanted_detector!='yolo' or t<=self.last_project or t not in self.info
                or t not in self.depth or t not in self.jpegs):
            return False
        self.boxes[t]=boxes
        self.publish_boxes(t,boxes)
        self.overlay(t)
        self.perceive(t)
        return True

    def overlay(self,t):
        image=Image.open(io.BytesIO(self.jpegs[t])).convert('RGB')
        draw=ImageDraw.Draw(image)
        for kind,confidence,(x,y,w,h) in self.boxes[t]:
            box=(x-w/2,y-h/2,x+w/2,y+h/2)
            draw.rectangle(box,outline='#54f0bf',width=2)
            draw.text((box[0],max(0,box[1]-12)),f'{kind} {confidence:.2f}',fill='#54f0bf')
        self.emit('/drone/camera/overlay/image',{'header':header(t,'camera_optical_frame'),'width':image.width,'height':image.height,
                  'encoding':'rgb8','is_bigendian':0,'step':image.width*3,'data':base64.b64encode(image.tobytes()).decode()})

    def map_tracks(self):
        transform=pose_matrix(self.fusion.applied)
        mapped = {}
        for key,source in self.tracker.tracks.items():
            # Flight logic gets the same Track instance and status updates as association.
            mapped[key]=(source.position.copy(),None if source.observation is None else source.observation.copy())
            source.position=transform_point(transform,source.position)
            if source.observation is not None:
                source.observation=transform_point(transform,source.observation)
        return mapped

    def tick(self):
        dt=min(self.now-self.last_tick,.25)
        self.last_tick=self.now
        self.fusion.predict(dt)
        pose=self.pose
        lcfg=self.config['localization']
        quality=self.fusion.quality(self.now)
        valid=self.slam.valid and self.now-self.slam.latest<.65 and self.now-self.range_time<.6
        if self.autostart and self.now-self.session_started>1. and valid and not self.mission.active and self.mission.outcome is None:
            self.mission.start(f'run-{self.epoch}')
        if self.now-self.last_map>.65 and self.slam.valid:
            correction=pose_matrix(self.fusion.applied)
            points=self.slam.points @ correction[:3,:3].T + correction[:3,3]
            self.mission.grid.update(points[::max(1,len(points)//2400)],.54)
            self.mission.grid_revision += 1
            self.last_map=self.now
            self.publish_cloud()
        tracks=self.tracker.tracks
        mapped=self.map_tracks()
        try:
            if self.mission.state=='SEARCH' and self.now-self.last_camera_observation<.35:
                self.mission.mark_coverage(pose,self.clearance+.18)
            cmd=self.mission.step(self.now,dt,pose,float(np.linalg.norm(self.slam.velocity)),self.clearance,quality,
                                  valid,tracks,self.raw_grip,self.fusion.accepted,
                                  3. if self.fusion.horizontal_sigma>lcfg['fail_horizontal_sigma_m'] else self.fusion.horizontal_sigma)
            if self.mission.manual and not self.mission.estop and not self.mission.paused and self.manual_cmd:
                t,m=self.manual_cmd
                if self.now-t < .25:
                    cmd.velocity=np.array([m['linear'][i] for i in 'xyz']);cmd.yaw=m['angular']['z']
            if not valid or self.mission.estop:
                cmd.velocity[:]=0;cmd.yaw=0.
            self.safety_limit(cmd,pose)
            self.emit('/drone/cmd_vel',{'header':header(self.now,'base_link'),'twist':{'linear':vector(cmd.velocity),'angular':vector([0.,0.,cmd.yaw])}})
            if cmd.grip:
                self.emit('/drone/gripper/command',{'header':header(self.now,'gripper_link'),'request_id':cmd.request,'command':cmd.grip})
            if self.mission.active and (self.sent_tilt!=cmd.tilt or int(self.now)%2==0 and int(self.now*20)%20==0):
                self.emit('/drone/camera/tilt_cmd',{'data':cmd.tilt});self.sent_tilt=cmd.tilt
            if self.now-self.last_ui>.17:
                self.last_ui=self.now
                self.telemetry(pose,quality,valid)
        finally:
            for key,(position,obs) in mapped.items():
                tracks[key].position=position;tracks[key].observation=obs

    def safety_limit(self,cmd,pose):
        if not np.isfinite(cmd.velocity).all() or not math.isfinite(cmd.yaw):
            cmd.velocity[:]=0.;cmd.yaw=0.;return
        fence=self.config['safety']['geofence_xy']
        velocity=yaw_rotation(pose[3])@cmd.velocity
        uncertainty=min(.65,self.fusion.horizontal_sigma*2.)
        margin=self.config['safety']['fence_margin_m']+.48+uncertainty
        for i in range(2):
            if pose[i]<fence[2*i]+margin and velocity[i]<0 or pose[i]>fence[2*i+1]-margin and velocity[i]>0:
                velocity[i]=0.
        if self.clearance+.18>=self.config['safety']['geofence_max_agl_m']:
            velocity[2]=min(0.,velocity[2])
        cmd.velocity=yaw_rotation(pose[3]).T@velocity
        # A close horizontal return in the travel sector stops translation.
        xy=np.linalg.norm(cmd.velocity[:2])
        if xy>.02 and len(self.scan)==360:
            heading=math.atan2(cmd.velocity[1],cmd.velocity[0]);center=int((heading+math.pi)*180/math.pi)%360
            if min(self.scan[(center+i)%360] for i in range(-24,25))<.60:
                cmd.velocity[:2]=0.

    def status(self):
        m=self.mission
        counts={f'{name}_{kind}s':0 for name in m.bins for kind in ('bottle','hammer')}
        for bin_id,kind,*_ in m.deliveries.values():
            counts[f'{bin_id}_{kind}s']+=1
        target=self.tracker.tracks.get(m.target_id)
        attached=bool(self.raw_grip.get('attached'))
        return {'header':header(self.now,'map'),'mission_id':m.mission_id,'epoch':self.epoch,'state':m.state,'paused':m.paused,
                'manual':m.manual,'estop_latched':m.estop,'detector':self.wanted_detector,'bin_policy':m.policy,'target_id':m.target_id,
                'payload_track_id':m.target_id if attached else '', 'payload_class':target.kind if attached and target else '',
                'payload_mass_kg':float(self.raw_grip.get('payload_mass_kg',0.)),'battery_fraction':m.battery,'coverage_fraction':m.coverage(self.pose),
                'reason':m.reason,**counts}

    def telemetry(self,pose,quality,valid):
        m=self.mission; state=self.status()
        self.emit('/mission/status',state)
        self.publish_ros_state(pose,quality,valid)
        model = Path(self.model_path) if self.model_path else None
        yolo_available = self.yolo is not None or bool(model and model.is_file() and model.with_suffix('.classes.json').is_file())
        self.json('/ui/telemetry',{'status':state,'pose':pose.tolist(),'quality':quality,'valid':valid,'fixes':self.fusion.accepted,
                                  'yolo_available':yolo_available,
                                  'fix_age':max(0.,self.now-self.fusion.last_fix),'sigma':self.fusion.horizontal_sigma,
                                  'boxes':self.last_boxes if self.now-max(self.boxes,default=-100)<.45 else [],
                                  'tracks':[[t.id,t.kind,t.status,*t.position.tolist()] for t in self.tracker.tracks.values()][-100:],
                                  'bins':{n:p.tolist() for n,p in m.bins.items()},'path':[p.tolist() for p in m.path],
                                  'map':(self.slam.points[::max(1,len(self.slam.points)//850)][:900] @ yaw_rotation(self.fusion.applied[3]).T + self.fusion.applied[:3]).round(2).tolist(),
                                  'coverage': [m.grid.center(c).tolist() for c in sorted(m.covered)]})

    def publish_cloud(self):
        data=np.asarray(self.slam.points,dtype='<f4').tobytes()
        self.emit('/slam/map',{'header':header(self.slam.latest,'odom'),'height':1,'width':len(self.slam.points),
                  'fields':[{'name':axis,'offset':i*4,'datatype':7,'count':1} for i,axis in enumerate('xyz')],
                  'is_bigendian':False,'point_step':12,'row_step':len(data),'data':base64.b64encode(data).decode(),'is_dense':True})

    def publish_ros_state(self,pose,quality,valid):
        fusion=self.fusion; m=self.mission; cloud=self.slam
        gps=diagonal([*np.diag(fusion.cov)[:3],1000.,1000.,fusion.cov[3,3]])
        local_cov=diagonal([.03,.03,.015,1000.,1000.,.02])
        h=header(self.now,'map')
        if cloud.valid:
            self.emit('/drone/odom',{'header':header(cloud.latest,'odom'),'child_frame_id':'base_link',
                      'pose':{'pose':pose_msg(cloud.pose),'covariance':local_cov},'twist':{'twist':{'linear':vector(yaw_rotation(cloud.pose[3]).T@cloud.velocity)}}})
            self.emit('/drone/pose',{'header':h,'pose':{'pose':pose_msg(pose),'covariance':gps}})
        self.emit('/localization/status',{'header':h,'quality':quality,'seconds_since_fix':max(0.,self.now-fusion.last_fix),
                  'horizontal_sigma_m':fusion.horizontal_sigma,'vertical_sigma_m':float(np.sqrt(fusion.cov[2,2])),
                  'yaw_sigma_rad':float(np.sqrt(fusion.cov[3,3])),'registration_rmse_m':cloud.rmse,'odometry_valid':bool(valid),
                  'fix_accepted':self.now-fusion.last_fix<.3 and fusion.accepted>0,'rejected_fixes':fusion.rejected,'reason': 'SLAM and station corrections' if valid else 'stale local odometry or range'})
        arrays=[]; markers=[{'header':h,'ns':'tracks','id':0,'action':3}]
        for idx,t in enumerate(self.tracker.tracks.values(),1):
            p=pose_msg([*t.position,0.])
            arrays.append({'track_id':t.id,'class_id':t.kind,'confidence':float(t.confidence),'pose':{'pose':p,'covariance':gps},
                           'last_seen':stamp(t.seen),'status':t.status,'grasp_attempts':t.attempts,'blacklisted':t.blacklisted})
            colors=[(.93,.65,.20),(.24,.90,.56),(.95,.82,.19),(.29,.68,.97),(.55,.55,.60),(.50,.85,.75)]
            markers.append(marker(h,'tracks',idx,p,colors[t.status],.22,t.id+' '+t.kind))
            if t.id==m.target_id:
                opt=transform_point(np.linalg.inv(pose_matrix(pose)@camera_mount(self.tilt)),t.position)
                self.emit('/cam_object_pose',{'header':header(self.now,'camera_optical_frame'),'pose':{'pose':pose_msg([*opt,0.]),'covariance':gps}})
        self.emit('/objects/tracks',{'header':h,'objects':arrays})
        self.emit('/visualization/objects',{'markers':markers})
        self.emit('/visualization/site',{'markers':[marker(h,'site',i,pose_msg([*p,0.]),color,.5,name) for i,(name,p,color) in enumerate([
                  ('ground_station',np.zeros(3),(.7,.9,.85)),('bin_a',m.bins['bin_a'],(.18,.8,.9)),('bin_b',m.bins['bin_b'],(.98,.57,.18))])]})
        self.emit('/mission/path',{'header':h,'poses':[{'header':h,'pose':pose_msg([*p,pose[2],0.])} for p in m.path]})
        a,b,c,d=m.config['search_bounds_xy']; r=m.grid.resolution; nx,ny=round((b-a)/r),round((d-c)/r)
        if nx>0 and ny>0:
            self.emit('/mission/coverage',{'header':h,'info':{'map_load_time':stamp(self.session_started),'resolution':r,'width':nx,'height':ny,
                      'origin':pose_msg([a,c,.02,0.])},'data':[100 if m.grid.cell([a+(i+.5)*r,c+(j+.5)*r]) in m.covered else 0 for j in range(ny) for i in range(nx)]})
        self.emit('/drone/battery',{'header':header(self.now,'base_link'),'percentage':m.battery,'present':True})

    def rpc(self,method,args,epoch=None):
        m=self.mission
        def response(ok,detail=''):
            return {'ok':bool(ok),'result':detail if ok else '', 'error':'' if ok else (detail or 'command rejected in current state')}
        if epoch is not None and int(epoch)!=self.epoch:
            return response(False,'scene epoch changed')
        if method=='start_mission':
            return response(m.start(str(args.get('mission_id','pilot-run'))))
        if method=='pause_mission':
            m.pause();return response(True,'Mission paused. Click Resume to continue.')
        if method=='resume_mission':
            if m.estop:
                return response(False,'E-stop latched: click Clear, then Resume.')
            self.emit('/drone/estop',{'data':False})
            self.json('/sim/control/request',{'operation':'resume','epoch':self.epoch,'request_id':'resume-mission'})
            return response(m.resume(),'Mission resumed.')
        if method=='estop':
            m.emergency_stop();self.emit('/drone/estop',{'data':True});return response(True)
        if method=='clear_estop':
            m.clear_estop();self.emit('/drone/estop',{'data':False});return response(True,'E-stop cleared. Click Resume to continue.')
        if method=='set_manual':
            m.manual=bool(args.get('enabled'));m.paused=not m.manual;return response(True)
        if method=='manual_grip':
            command=args.get('command')
            if not m.manual or m.estop or command not in ('open','close'):
                return response(False,'manual flight and a cleared latch are required')
            m.serial += 1
            self.emit('/drone/gripper/command',{'header':header(self.now,'gripper_link'),'request_id':f'pilot-{m.serial}','command':command})
            return response(True)
        if method=='set_bin_policy':
            p,b=str(args.get('policy','')),str(args.get('selected_bin',''))
            if p not in ('by_class','nearest','ui_select') or p=='ui_select' and b not in m.bins:
                return response(False,'invalid bin policy or operator bin')
            m.policy,m.selected_bin=p,b;return response(True)
        if method=='select_target':
            return response(m.select_target(self.tracker.tracks,str(args.get('track_id',''))))
        if method=='set_detector':
            try:
                self.set_detector(args['mode']);return response(True)
            except (KeyError,ValueError,ImportError,OSError,RuntimeError) as error:
                return response(False,str(error))
        if method in ('reset_scene','pause_sim','resume_sim','step_sim'):
            if method=='reset_scene' and (m.estop or m.active or self.raw_grip.get('attached')):
                return response(False,'land, finish the mission and clear E-stop before resetting')
            operations={'reset_scene':'reset','pause_sim':'pause','resume_sim':'resume','step_sim':'step'}
            self.json('/sim/control/request',{'operation':operations[method],'epoch':self.epoch,'request_id':method,**args});return response(True,'sent to simulator')
        return response(False,'unknown operation')


def marker(h,ns,i,p,color,diameter,text):
    return {'header':h,'ns':ns,'id':i,'type':9,'action':0,'pose':p,'scale':vector([diameter]*3),
            'color':dict(zip('rgba',(*color,1.))), 'text':text}
