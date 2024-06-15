from setuptools import find_packages
from setuptools import setup

setup(
    name='costmap_converter_msgs',
    version='0.1.2',
    packages=find_packages(
        include=('costmap_converter_msgs', 'costmap_converter_msgs.*')),
)
