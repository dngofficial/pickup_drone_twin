import io
import json

import numpy as np
import pytest
from PIL import Image

from urc_core.geometry import header, stamp
from urc_core.runtime import Twin


@pytest.mark.parametrize('asynchronous',[False,True])
def test_yolo_waits_for_rgb_depth_and_lidar_pose(config,asynchronous):
    sent = []
    requested = []
    def request(t,jpeg):
        requested.append((t,jpeg))
        return True
    twin = Twin(config, lambda topic, value: sent.append((topic,value)),
                inference_request=request if asynchronous else None)

    class Detector:
        calls = 0
        def infer(self, jpeg):
            self.calls += 1
            return [('bottle',.95,(320.,180.,60.,50.))]

    twin.wanted_detector = 'yolo'
    twin.yolo = Detector()
    image = io.BytesIO()
    Image.new('RGB',(640,360)).save(image,format='JPEG')
    h = header(1.,'camera_optical_frame')
    twin.on_message('/drone/camera/camera_info',{'header':h,'k':[320.,0.,319.5,0.,320.,179.5,0.,0.,1.]})
    twin.on_message('/drone/camera/image/compressed',{'header':h,'data':image.getvalue()})
    assert twin.yolo.calls == 0, 'RGB-only frames must not consume inference time'
    twin.on_message('/drone/camera/depth/image_raw',{
        'header':h,'encoding':'16UC1','step':1280,'width':640,'height':360,'is_bigendian':0,
        'data':np.full((360,640),2000,dtype='<u2').tobytes()})
    assert twin.yolo.calls == 0, 'Projection needs a matching pose before inference'
    twin.slam.pose[2] = 3.
    twin.slam.update = lambda _packet, _now: True
    twin.on_message('/drone/lidar/packet',{'data':json.dumps({
        'version':2,'epoch':1,'seq':1,'stamp':stamp(1.)})})
    if asynchronous:
        assert twin.yolo.calls == 0 and len(requested)==1
        # Sensor/clock processing continues while the model job is pending.
        twin.on_message('/clock',{'clock':stamp(1.1)})
        assert twin.now == 1.1
        assert twin.complete_inference(requested[0][0],twin.yolo.infer(requested[0][1]))
    assert twin.yolo.calls == 1
    assert twin.last_project == 1.
    detections = [value for topic,value in sent if topic=='/perception/detections_3d']
    assert len(detections) == 1 and len(detections[0]['detections']) == 1
    assert detections[0]['detections'][0]['results'][0]['hypothesis']['class_id'] == 'bottle'
    # An unpaired subsequent RGB frame must leave the completed observation intact.
    twin.on_message('/drone/camera/image/compressed',{'header':header(1.1,'camera_optical_frame'),'data':image.getvalue()})
    assert twin.yolo.calls == 1
    assert twin.bad_input == 0
