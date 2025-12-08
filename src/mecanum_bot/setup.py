import os
from glob import glob
from setuptools import find_packages, setup


package_name = 'mecanum_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 1. COPY LAUNCH FILES
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. COPY CONFIG FILES (YAML)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # 3. COPY DESCRIPTION FILES (URDF/XACRO)
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosuser',
    maintainer_email='rosuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
   entry_points={
        'console_scripts': [
            'hardware_driver = mecanum_bot.hardware_driver:main',
            'data_controller = mecanum_bot.data_controller_node:main',
            'pid = mecanum_bot.pid:main', 
        ],
    },
)