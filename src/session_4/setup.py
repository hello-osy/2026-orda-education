from pathlib import Path
from setuptools import setup

data_files = [('share/ament_index/resource_index/packages', ['resource/session_4']),
              ('share/session_4', ['package.xml']),
              ('share/session_4/student', ['student/driving.py'])]
for folder in ('launch', 'config', 'worlds', 'models', 'weights'):
    for parent in sorted({p.parent for p in Path('common', folder).rglob('*') if p.is_file()}):
        files = [str(p) for p in sorted(parent.iterdir()) if p.is_file() and p.suffix != '.pyc']
        if files and '__pycache__' not in str(parent):
            data_files.append((f'share/session_4/{parent.relative_to("common")}', files))

setup(name='session_4', version='0.1.0', packages=['session_4'], package_dir={'session_4': 'common'},
      data_files=data_files, install_requires=['setuptools'], zip_safe=False,
      maintainer='ORDA', maintainer_email='hello_osy@naver.com', license='Proprietary',
      description='ORDA Gazebo autonomous driving and four-role Git collaboration',
      entry_points={'console_scripts': [
          'sim_control = session_4.sim_control_node:main',
          'traffic_signal = session_4.traffic_signal:main',
          'yolo_node = session_4.yolo_node:main',
          'debug_viewer = session_4.debug_viewer:main',
          'manual_drive = session_4.manual_drive:main',
          'collect_data = session_4.recording:main',
      ]})
