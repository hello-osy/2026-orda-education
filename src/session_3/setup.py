from pathlib import Path
from setuptools import find_packages, setup

data_files = [('share/ament_index/resource_index/packages', ['resource/session_3']),
              ('share/session_3', ['package.xml'])]
for folder in ('launch', 'config', 'worlds', 'models'):
    for parent in sorted({p.parent for p in Path(folder).rglob('*') if p.is_file()}):
        data_files.append((f'share/session_3/{parent}',
                           [str(p) for p in sorted(parent.iterdir()) if p.is_file()]))

setup(name='session_3', version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=data_files, install_requires=['setuptools'], zip_safe=False,
      maintainer='ORDA', maintainer_email='hello_osy@naver.com', license='Proprietary',
      description='ORDA ROS 2 topics, Gazebo car and steering-only hardware exercises',
      entry_points={'console_scripts': [
          'camera_node = session_3.camera_node:main',
          'lidar_node = session_3.lidar_node:main',
          'motor_node = session_3.motor_node:main',
          'sim_control = session_3.sim_control_node:main',
          'camera_viewer = session_3.camera_viewer:main',
          'lidar_viewer = session_3.lidar_viewer:main',
      ]})
