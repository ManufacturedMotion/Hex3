from setuptools import setup
import os
from glob import glob

package_name = 'hexapod_msgs'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        # Required so ROS2 can find the package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # 👇 THIS is the important part for message packages
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Hexapod shared message definitions',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)