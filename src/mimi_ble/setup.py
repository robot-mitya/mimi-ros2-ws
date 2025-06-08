from setuptools import find_packages, setup

package_name = 'mimi_ble'

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
    maintainer='DmitryDzz',
    maintainer_email='DzakhovDS@gmail.com',
    description='This package contains the mimi_ble_node, a Bluetooth gateway node responsible for wireless communication between the Mimi robot and external applications.',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)
