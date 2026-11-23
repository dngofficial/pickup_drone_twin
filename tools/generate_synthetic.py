#!/usr/bin/env python3
import argparse
from pathlib import Path
import subprocess


def main():
    parser=argparse.ArgumentParser(description='Render Godot RGB and instance-mask labels')
    parser.add_argument('--godot',default='godot');parser.add_argument('--seeds',default='0:200')
    parser.add_argument('--frames',type=int,default=12);parser.add_argument('--output',type=Path,required=True)
    a=parser.parse_args();lo,hi=map(int,a.seeds.split(':'))
    project=Path(__file__).resolve().parents[1]/'godot'
    subprocess.run([a.godot,'--path',str(project),'--audio-driver','Dummy','--script','res://scripts/synthetic_export.gd','--',
                    f'--first={lo}',f'--last={hi}',f'--frames={a.frames}',f'--output={a.output.resolve()}'],check=True)


if __name__=='__main__': main()
