from pathlib import Path
import shutil
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml
from urc_core.geometry import quaternion
from urc_core.runtime import Twin


def launch(context):
    share=Path(get_package_share_directory('urc_bringup'))
    val=lambda name:LaunchConfiguration(name).perform(context)
    config=val('config') or str(share/'config/defaults.yaml')
    conf=yaml.safe_load(Path(config).read_text())
    sim=val('sim')
    if sim not in ('godot','gazebo'):
        raise ValueError('sim must be godot or gazebo')
    bridge=Path(get_package_share_directory('rosbridge_server'))/'launch/rosbridge_websocket_launch.xml'
    nodes=[IncludeLaunchDescription(AnyLaunchDescriptionSource(str(bridge)),launch_arguments={'address':val('bridge_address'),'port':val('port')}.items()),
           Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'use_sim_time':True,
                'robot_description':ParameterValue(Command(['xacro ',str(share/'urdf/drone.urdf.xacro')]),value_type=str)}]),
           Node(package='urc_bringup',executable='twin_node',output='screen',parameters=[{'config':config,'use_sim_time':True,
                'detector':val('detector'),'model':val('model'),'autostart':val('autostart').lower()=='true'}])]
    for child,xyz in [('ground_station',[0.,0.,0.]),*( (key,conf['scene'][key+'_map_xyz']) for key in ('bin_a','bin_b'))]:
        xyz_args=[part for axis,v in zip('xyz',xyz) for part in ('--'+axis,str(v))]
        nodes.append(Node(package='tf2_ros',executable='static_transform_publisher',name=f'frame_{child}',
                          arguments=xyz_args+['--frame-id','map','--child-frame-id',child]))
    for key,pose in Twin.default_tags(conf).items():
        q=quaternion(pose[:3,:3]); translation = [part for axis,v in zip('xyz',pose[:3,3]) for part in ('--'+axis,str(v))]
        rotation = [part for axis,v in q.items() for part in ('--q'+axis,str(v))]
        nodes.append(Node(package='tf2_ros',executable='static_transform_publisher',name=f'tag_frame_{key}',
                          arguments=translation+rotation+['--frame-id','map','--child-frame-id',f'tag_{key}']))
    if val('rviz').lower()=='true':
        nodes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/twin.rviz')],parameters=[{'use_sim_time':True}]))
    if sim=='gazebo':
        if not shutil.which('gz') or not val('world'):
            raise RuntimeError('Install Gazebo Harmonic and pass world:=ABSOLUTE_SDF from tools/generate_gazebo.py')
        nodes += [ExecuteProcess(cmd=['gz','sim','-r',val('world')],output='screen'),
                  Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(share/'config/ros_gz_bridge.yaml')}]),
                  Node(package='urc_bringup',executable='gazebo_adapter',parameters=[{'manifest':str(Path(val('world')).with_suffix('.manifest.json')),'use_sim_time':True}])]
    return nodes


def generate_launch_description():
    args={'sim':'godot','detector':'oracle','model':'','rviz':'false','autostart':'false','world':'','config':'','bridge_address':'127.0.0.1','port':'9090'}
    return LaunchDescription([*(DeclareLaunchArgument(k,default_value=v) for k,v in args.items()),OpaqueFunction(function=launch)])
