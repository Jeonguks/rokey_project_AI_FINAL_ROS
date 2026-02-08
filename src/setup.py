import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'fight_fire'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 'launch' 폴더 안에 있는 모든 런치 파일(*launch.py 등)을 설치 폴더의 share/패키지명/launch 로 복사함
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='seoklee.rokey@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task_controller = fight_fire.task_controller_node:main',
            'action_controller = fight_fire.action_controller_node:main'
            'perception_node = fight_fire.perception_node:main',
        ],
    },
)