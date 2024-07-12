from setuptools import find_packages, setup

package_name = 'braking_node'

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
    maintainer='Yisreal',
    maintainer_email='nagual1414@gmail.com',
    description='TODO: Braking node for ROS 2 Humble Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'braking_node = braking_node.braking_node:main',
            'braking_teleop_keyboard = braking_node.braking_teleop_keyboard:main'
        ],
    },
)
