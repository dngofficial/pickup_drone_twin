#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import shutil


def main():
    p=argparse.ArgumentParser(description='Export a trained bottle/hammer YOLOv8 model')
    p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    from ultralytics import YOLO
    import onnxruntime as ort
    import numpy as np
    import torch
    model=YOLO(str(a.checkpoint));names=[model.names[i] for i in sorted(model.names)]
    if names!=['bottle','hammer']:p.error(f'expected bottle/hammer class order, got {names}')
    source=Path(model.export(format='onnx',imgsz=640,opset=17,dynamic=False,half=False,nms=False,simplify=False,device='cpu'))
    session=ort.InferenceSession(str(source),providers=['CPUExecutionProvider']);key=session.get_inputs()[0].name
    samples=np.random.default_rng(52).random((1,3,640,640),dtype=np.float32)
    out=session.run(None,{key:samples})[0]
    with torch.inference_mode():
        native=model.model.eval()(torch.from_numpy(samples))
        native=(native[0] if isinstance(native,(tuple,list)) else native).cpu().numpy()
    np.testing.assert_allclose(native,out,rtol=.02,atol=.012)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    if a.output.resolve()!=source.resolve(): shutil.copyfile(source,a.output)
    a.output.with_suffix('.classes.json').write_text(json.dumps({'names':names,'input':640,'format':'YOLOv8 raw head','opset':17},indent=2))
    print(a.output)


if __name__=='__main__': main()
