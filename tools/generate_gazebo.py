#!/usr/bin/env python3
"""Generate a Harmonic SDF world and terrain mesh from the Godot site manifest."""
import argparse
import json
import math
import subprocess
from pathlib import Path
from xml.etree import ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]


def tag(parent,element,text=None,**attrs):
    child=ET.SubElement(parent,element,attrs)
    if text is not None: child.text=str(text)
    return child


def xyz(values):
    return ' '.join(str(v) for v in values)


def ground(manifest,x,y):
    s=max(0,min(1,(x-2.6)/5.4));fade=s*s*(3-2*s);p=manifest['terrain_phase']
    return fade*manifest['terrain_amplitude']*(math.sin(.31*x+p)+.6*math.cos(.42*y-p)+.32*math.sin(.68*(x+y)))


def geometry(link,name,kind,dimensions,pose,color):
    for group in ('collision','visual'):
        component=tag(link,group,name=name+'_'+group);tag(component,'pose',xyz(pose))
        shape=tag(tag(component,'geometry'),kind)
        if kind=='box': tag(shape,'size',xyz(dimensions))
        elif kind=='mesh': tag(shape,'uri',dimensions)
        else: tag(shape,'radius',dimensions[0]);tag(shape,'length',dimensions[1])
        if group=='visual':
            material=tag(component,'material');tag(material,'ambient',color);tag(material,'diffuse',color)


def model(world,name,pose,static=False,link_name='link',mass=.7):
    m=tag(world,'model',name=name);tag(m,'pose',xyz(pose));tag(m,'static',str(static).lower())
    link=tag(m,'link',name=link_name)
    if not static:
        inertia=tag(link,'inertial');tag(inertia,'mass',mass)
        values=tag(inertia,'inertia')
        for k in ('ixx','iyy','izz'): tag(values,k,max(.0004,mass*.025))
    return m,link


def create(data,output):
    output.parent.mkdir(parents=True,exist_ok=True)
    mesh=output.with_name('terrain.obj')
    length,width=113,113
    with mesh.open('w') as file:
        for i in range(length):
            for j in range(width):
                x,y=-12+i*.5,-28+j*.5
                file.write(f'v {x} {y} {ground(data,x,y):.7f}\n')
        for i in range(length-1):
            for j in range(width-1):
                a=i*width+j+1;b=a+width
                file.write(f'f {a} {b} {a+1}\nf {b} {b+1} {a+1}\n')
    sdf=ET.Element('sdf',{'version':'1.9'});w=tag(sdf,'world',name='urc_terrain')
    physics=tag(w,'physics',name='normal',type='ignored');tag(physics,'max_step_size',.004);tag(physics,'real_time_factor',1.)
    for plugin,typ in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors')]:
        node=tag(w,'plugin',filename=f'gz-sim-{plugin}-system',name='gz::sim::systems::'+typ)
        if typ=='Sensors':tag(node,'render_engine','ogre2')
    light=tag(w,'light',type='directional',name='sun');tag(light,'direction','-0.5 0.2 -1');tag(light,'diffuse','0.9 0.84 0.71 1')
    _,terrain=model(w,'terrain',[0]*6,True);geometry(terrain,'soil','mesh',str(mesh.resolve()),[0]*6,'0.59 0.40 0.22 1')
    for i,rock in enumerate(data['rocks']):
        x,y=rock['xy'];h=rock['height_m'];r=rock['radius_m']
        _,link=model(w,f'rock_{i}',[x,y,ground(data,x,y)+h*.46,0,0,rock['yaw']],True)
        geometry(link,'rock','box',[1.5*r,1.4*r,h],[0]*6,'0.41 0.32 0.26 1')
    _,home=model(w,'ground_station',[0]*6,True)
    geometry(home,'pad','box',[2.2,2.2,.068],[0,0,-.015,0,0,0],'0.16 0.22 0.24 1')
    geometry(home,'mast','box',[.09,.09,2.1],[-.64,1.38,1.05,0,0,0],'0.34 0.41 0.44 1')
    for dest,color in [('bin_a','0.13 0.61 0.67 1'),('bin_b','0.85 0.42 0.16 1')]:
        _,link=model(w,dest,[*data[dest+'_map_xyz'],0,0,0],True)
        geometry(link,'base','box',[1.36,1.36,.06],[0,0,.03,0,0,0],color)
        for i,s in enumerate((-1,1)):
            geometry(link,f'front{i}','box',[.08,1.36,.6],[s*.64,0,.3,0,0,0],color)
            geometry(link,f'side{i}','box',[1.2,.08,.6],[0,s*.64,.3,0,0,0],color)
    for mark in data['tag_poses_map']:
        p=mark['position'];_,panel=model(w,'tag_'+str(mark['tag_id']),[p['x'],p['y'],p['z'],0,0,0],True)
        geometry(panel,'panel','box',[.27,.27,.013],[0]*6,'0.9 0.88 0.73 1')
    for thing in data['objects']:
        bottle=thing['class_id']=='bottle'; x,y,z=thing['xyz']
        _,obj=model(w,thing['id'],[x,y,z,math.pi/2 if bottle and thing['sideways'] else 0,0,thing['yaw']],False,'object',.15 if bottle else .55)
        if bottle: geometry(obj,'bottle','cylinder',[.044,.28],[0]*6,'0.08 0.67 0.86 1')
        else:
            geometry(obj,'handle','box',[.275,.04,.045],[-.045,0,0,0,0,0],'0.87 0.41 0.13 1')
            geometry(obj,'head','box',[.07,.19,.055],[.135,0,0,0,0,0],'0.32 0.37 0.40 1')
    drone,body=model(w,'pickup_drone',data['initial_base_map_xyz_yaw'][:3]+[0,0,0],False,'base_link',1.6)
    tag(body,'gravity','false')
    geometry(body,'fuselage','box',[.59,.47,.22],[0,0,-.04,0,0,0],'0.23 0.32 0.38 1')
    geometry(body,'grasp_visual','box',[.12,.12,.12],[0,0,-.49,0,0,0],'0.23 0.73 0.68 1')
    for i,(x,y) in enumerate([(-.38,-.38),(-.38,.38),(.38,-.38),(.38,.38)]):
        geometry(body,'prop_'+str(i),'cylinder',[.17,.035],[x,y,.06,0,0,0],'0.13 0.18 0.22 1')
    tag(drone,'plugin',filename='liburc_gz_system.so',name='urc::FlightSystem')
    lidar=tag(body,'sensor',name='lidar',type='gpu_lidar');tag(lidar,'pose','0 0 0.18 0 0 0');tag(lidar,'topic','/gz/urc/lidar');tag(lidar,'update_rate',10);tag(lidar,'always_on','true')
    ray=tag(lidar,'lidar');scan=tag(ray,'scan')
    for axis,samples,left,right in [('horizontal',360,-math.pi,math.pi),('vertical',10,-.87,.87)]:
        axis_tag=tag(scan,axis);tag(axis_tag,'samples',samples);tag(axis_tag,'resolution',1);tag(axis_tag,'min_angle',left);tag(axis_tag,'max_angle',right)
    bounds=tag(ray,'range');tag(bounds,'min',.08);tag(bounds,'max',40);tag(bounds,'resolution',.01)
    rgb=tag(body,'sensor',name='rgbd',type='rgbd_camera');tag(rgb,'pose','0.2 0 -0.15 0 1.5707963 0');tag(rgb,'topic','/gz/urc/rgbd');tag(rgb,'update_rate',10);tag(rgb,'always_on','true')
    camera=tag(rgb,'camera');tag(camera,'horizontal_fov',1.596851);image=tag(camera,'image');tag(image,'width',640);tag(image,'height',360);tag(image,'format','R8G8B8')
    clip=tag(camera,'clip');tag(clip,'near',.035);tag(clip,'far',60.)
    ET.indent(sdf)
    ET.ElementTree(sdf).write(output,encoding='utf-8',xml_declaration=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--godot',default='godot');p.add_argument('--seed',type=int,default=42)
    p.add_argument('--output',type=Path,required=True); a=p.parse_args();output=a.output.resolve();output.parent.mkdir(parents=True,exist_ok=True)
    manifest=output.with_suffix('.manifest.json')
    subprocess.run([a.godot,'--headless','--path',str(ROOT/'godot'),'--',f'--seed={a.seed}',f'--manifest={manifest}'],check=True)
    create(json.loads(manifest.read_text()),output)
    print(output)


if __name__=='__main__': main()
