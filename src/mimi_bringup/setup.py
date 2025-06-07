import os

from setuptools import find_packages, setup

package_name = 'mimi_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # data_files=[
    #     ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    data_files=[
        (f'share/{package_name}/launch', [os.path.join('launch', 'launch.py')]),
        (f'share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'launch',
        'launch_ros'
    ],
    zip_safe=True,
    maintainer='DmitryDzz',
    maintainer_email='DzakhovDS@gmail.com',
    description='Launch and system startup logic for the Mimi robot. This package defines launch files that bring up the robot''s core functionality including sensor nodes, communication interfaces, and system parameters.',
    license='MIT',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
