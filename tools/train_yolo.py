#!/usr/bin/env python3
import argparse
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description='Fine-tune YOLOv8n on real and labelled Godot images')
    p.add_argument('--data',type=Path,required=True);p.add_argument('--device',default='cpu')
    p.add_argument('--epochs',type=int,default=45);p.add_argument('--batch',type=int,default=8)
    p.add_argument('--workers',type=int,default=2,help='Data-loader workers; keep low on Windows to limit RAM use')
    p.add_argument('--weights',default='yolov8n.pt');p.add_argument('--seed',type=int,default=42)
    a=p.parse_args()
    from ultralytics import YOLO
    training=YOLO(a.weights)
    training.train(data=str(a.data.resolve()),epochs=a.epochs,batch=a.batch,imgsz=640,device=a.device,
                   project=str(Path('runs').resolve()),name='train',seed=a.seed,deterministic=True,workers=a.workers)
    model=YOLO(training.trainer.best)
    for domain in ('real','synthetic'):
        source=a.data.with_name(domain+'.yaml')
        if source.exists():
            performance=model.val(data=str(source.resolve()),imgsz=640,device=a.device,workers=a.workers)
            print(domain,performance.results_dict)


if __name__=='__main__': main()
