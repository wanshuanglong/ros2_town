from setuptools import setup
from glob import glob
import os

package_name = 'village_li'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    # maintainer='ros2',
    maintainer='ros2',
    # maintainer_email='sangxin2014@sina.com',
    maintainer_email='wanshuanglong@mychery.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "li4_node = village_li.li4:main",
            "li3_node = village_li.li3:main"
        ],
    },
)
