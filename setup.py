from setuptools import setup, find_packages
from glob import glob
import os


def package_files(directory):
    paths = []
    for (path, _dirs, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

package_name = 'imav_2017'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.xml'))),
        ('share/' + package_name + '/meshes', package_files('meshes')),
        ('share/' + package_name + '/urdf', package_files('urdf')),
        ('share/' + package_name + '/worlds', package_files('worlds')),
        ('share/' + package_name + '/scripts', package_files('scripts')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vince',
    maintainer_email='vince@todo.todo',
    description='IMAV 2017 Virtual Challenge - ROS2 Migration',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ps2cmd_vel = scripts.ps2cmd_vel:main',
            'quadrotor_controller = scripts.quadrotor_controller:main',
        ],
    },
)
