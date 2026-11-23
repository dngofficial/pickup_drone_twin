#!/usr/bin/env python3
import argparse
import os
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description='Download the licensed URC Hammer-Bottle dataset')
    parser.add_argument('--version',type=int,default=1);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    key=os.environ.get('ROBOFLOW_API_KEY')
    if not key:
        parser.error('export your own ROBOFLOW_API_KEY locally')
    from roboflow import Roboflow
    try:
        project=Roboflow(api_key=key).workspace('urc-cvc4i').project('hammer-bottle-l3zmd-odqnx')
        project.version(args.version).download('yolov8',location=str(args.output.resolve()),overwrite=False)
    except Exception as exc:
        raise SystemExit(f'Roboflow download failed ({type(exc).__name__}). Check key access and project/version in Roboflow.') from None


if __name__=='__main__': main()
