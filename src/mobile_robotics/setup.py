from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mobile_robotics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # índice para ament_python
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package.xml para el paquete
        ('share/' + package_name, ['package.xml']),
        # todos los launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # todos los YAML de parámetros
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arturo',
    maintainer_email='arturo@todo.todo',
    description='Navegación en PuzzleBot con Odometry y tiempo estimado',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odometry_node   = mobile_robotics.odometry_node:main',
          #  'controller_node = mobile_robotics.controller_node:main',
           # 'path_generator  = mobile_robotics.path_generator:main',
        ],
    },
)
