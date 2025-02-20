from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'autonomous_rov'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),        
        (os.path.join('share', package_name, 'config'), glob('config/*.config.yaml')),
        (os.path.join('share', package_name, 'lib'), glob('*.py')),                                                
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vincent',
    maintainer_email='vincent@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'visual_servoing = autonomous_rov.visual_servoing_essai:main',
        'state_machine = autonomous_rov.state_machine_visual_servo:main',
        'video = autonomous_rov.video:main',
        'image_processing_tracker = autonomous_rov.image_processing_tracker:main',
        'image_processing_yolo= autonomous_rov.image_processing_yolo:main',
        'listenerMIR = autonomous_rov.listenerMIR:main',
        'listenerVS = autonomous_rov.listener_visual_servoing:main'
        ],       
        
    },
)
