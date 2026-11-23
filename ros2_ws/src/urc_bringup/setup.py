from glob import glob
from setuptools import setup

setup(name='urc_bringup',version='0.1.0',packages=['urc_bringup'],
      data_files=[('share/ament_index/resource_index/packages',['resource/urc_bringup']),('share/urc_bringup',['package.xml']),
                 *[(f'share/urc_bringup/{directory}', glob(f'{directory}/*')) for directory in ('launch','urdf','rviz','config')]],
      install_requires=['setuptools'],license='AGPL-3.0-only',
      entry_points={'console_scripts':['twin_node=urc_bringup.node:main','gazebo_adapter=urc_bringup.gazebo_adapter:main']})
