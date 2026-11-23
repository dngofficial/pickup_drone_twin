from xml.etree import ElementTree as ET
from generate_gazebo import create


def test_world_generator_creates_geometry_and_sensors(tmp_path):
    site={'terrain_phase':0.4,'terrain_amplitude':.1,'rocks':[], 'bin_a_map_xyz':[-2,2.5,0],
          'bin_b_map_xyz':[-2,-2.5,0],'initial_base_map_xyz_yaw':[0,0,.65,0], 'tag_poses_map':[],
          'objects':[{'id':'bottle_1','class_id':'bottle','xyz':[8.,-1.,.15],'sideways':True,'yaw':.2}]}
    path=tmp_path/'site.sdf';create(site,path)
    world=ET.parse(path).getroot().find('world'); names={x.attrib['name'] for x in world.findall('model')}
    assert {'terrain','ground_station','pickup_drone','bottle_1','bin_a','bin_b'}<=names
    drone=world.find("./model[@name='pickup_drone']")
    assert {s.attrib['type'] for s in drone.findall('.//sensor')}=={'gpu_lidar','rgbd_camera'}
    assert (tmp_path/'terrain.obj').stat().st_size>50_000
