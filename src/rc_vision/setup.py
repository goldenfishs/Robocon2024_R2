from setuptools import find_packages, setup

package_name = 'rc_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1683502971@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_quqiu = rc_vision.pub_quqiu:main',
            'pub_zhaokuang = rc_vision.pub_zhaokuang:main',
            'pub_quzhao = rc_vision.pub_quzhao:main',
            'sub_stop_quqiu = rc_vision.sub_stop_quqiu:main',
            'pub_move_mode = rc_vision.pub_move_mode:main',
            'pub_move_mode2= rc_vision.pub_move_mode2:main'
        ],
    },
)
