from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_px4_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zeyuan wang',
    maintainer_email='aircn33450@126.com',
    description='A ROS2 package for multi-agent formation control',
    license='CY',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 入口名 = 模块路径:函数名
            'arm_control = my_px4_demo.arm_control:main',
            'formation_demo = my_px4_demo.formation_demo:main',
            'formation_etm = my_px4_demo.formation_etm:main',
            'formation_node = my_px4_demo.formation_node:main',
            'etm_adjuster = my_px4_demo.etm_adjuster:main'
        ],
    },
)
