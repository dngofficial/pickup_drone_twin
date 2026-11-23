"""YOLOv8 detection head decoder and CPU fallback."""
import io
from pathlib import Path
import json
import numpy as np
from PIL import Image


class OnnxDetector:
    def __init__(self, weights, size=640, threshold=.5, nms_iou=.45):
        weights = Path(weights)
        if not weights.is_file():
            raise ValueError('YOLO model missing. Export a trained checkpoint to weights/hammer_bottle_yolov8n.onnx first.')
        manifest = weights.with_suffix('.classes.json')
        if not manifest.is_file():
            raise ValueError('YOLO class manifest missing. Export the .classes.json sidecar with the model.')
        metadata = json.loads(manifest.read_text(encoding='utf-8'))
        if metadata['names'] != ['bottle', 'hammer']:
            raise ValueError('expected trained bottle/hammer checkpoint and class manifest')
        import onnxruntime as ort
        providers = ['CPUExecutionProvider']
        if 'CUDAExecutionProvider' in ort.get_available_providers():
            providers.insert(0, 'CUDAExecutionProvider')
        options = ort.SessionOptions()
        options.intra_op_num_threads = 4
        options.inter_op_num_threads = 1
        options.add_session_config_entry('session.intra_op.allow_spinning', '0')
        self.session = ort.InferenceSession(str(weights), sess_options=options, providers=providers)
        self.input = self.session.get_inputs()[0].name
        self.size, self.threshold, self.nms_iou = size, threshold, nms_iou

    def infer(self, jpeg):
        im = Image.open(io.BytesIO(jpeg)).convert('RGB')
        scale = min(self.size/im.width, self.size/im.height)
        width, height = round(im.width*scale), round(im.height*scale)
        px, py = (self.size-width)//2, (self.size-height)//2
        padded = Image.new('RGB', (self.size,self.size), (114,114,114))
        padded.paste(im.resize((width,height), Image.Resampling.BILINEAR), (px,py))
        x = np.asarray(padded, dtype=np.float32).transpose(2,0,1)[None] / 255.
        data = self.session.run(None, {self.input: x})[0][0]
        if data.shape[0] != 6:
            raise ValueError(f'expected raw YOLOv8 two-class head [1,6,N], received {data.shape}')
        rows = data.T
        scores = rows[:,4:].max(axis=1)
        choices = np.argsort(scores)[::-1]
        kept = []
        for idx in choices[:500]:
            if scores[idx] < self.threshold:
                break
            cx,cy,w,h = rows[idx,:4]
            kind = ['bottle','hammer'][rows[idx,4:].argmax()]
            box = ((cx-px)/scale, (cy-py)/scale, w/scale, h/scale)
            if not any(k==kind and iou(box,b)>self.nms_iou for k,_,b in kept):
                kept.append((kind,float(scores[idx]),tuple(map(float,box))))
        return kept


def iou(a, b):
    def edges(box):
        x,y,w,h = box
        return x-w/2,y-h/2,x+w/2,y+h/2
    ax,ay,ar,ab = edges(a)
    bx,by,br,bb = edges(b)
    o = max(0,min(ar,br)-max(ax,bx))*max(0,min(ab,bb)-max(ay,by))
    return o/max(a[2]*a[3]+b[2]*b[3]-o,1e-9)
