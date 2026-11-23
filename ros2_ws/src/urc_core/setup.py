from setuptools import setup

setup(name='urc_core', version='0.1.0', packages=['urc_core'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/urc_core']),
                  ('share/urc_core', ['package.xml'])],
      install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML', 'Pillow'], license='AGPL-3.0-only')
