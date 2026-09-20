from setuptools import find_packages, setup
setup(
    name='session_1', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/session_1']),
        ('share/session_1', ['package.xml']),
        ('share/session_1/models', ['models/lane_pidnet_s.pt', 'models/dataset_info.json']),
        ('share/session_1/licenses', ['session_1/vendor/LICENSE']),
    ],
    install_requires=['setuptools'], zip_safe=False,
    maintainer='ORDA', maintainer_email='hello_osy@naver.com',
    description='PIDNet, scan line, PID before/after educational visualization',
    license='Proprietary; vendored PIDNet MIT',
    entry_points={'console_scripts': [
        'classroom = session_1.classroom:main',
        'segmentation_view = session_1.segmentation_view:main',
        'scan_line_view = session_1.scan_line_view:main',
        'pid_view = session_1.pid_view:main',
        'reference_color_filter_view = session_1.reference_color_filter_view:main',
        'reference_sliding_window_view = session_1.reference_sliding_window_view:main',
        'pipeline_after_view = session_1.pipeline_after_view:main',
    ]},
)
