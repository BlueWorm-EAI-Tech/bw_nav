from setuptools import setup

setup(
    name='bw_planning_zones', version='0.1.0', packages=['bw_planning_zones'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/bw_planning_zones']),
        ('share/bw_planning_zones', ['package.xml', 'README.md']),
        ('share/bw_planning_zones/launch', ['launch/planning_zones.launch.py']),
    ],
    maintainer='BW Robotics', maintainer_email='dev@bw.local', license='MIT',
    description='Standalone polygon editing and planning-zone mask publishing',
    entry_points={'console_scripts': [
        'planning_zone_server = bw_planning_zones.server:main',
        'planning_zone_editor = bw_planning_zones.editor:main',
    ]},
)
