from setuptools import find_packages, setup

package_name = 'ackermann_trajectory_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anthony Santacruz',
    maintainer_email='anthony@example.com',
    description='Controlador de trayectoria Pure Pursuit para robot Ackermann en ROS 2 Jazzy',
    license='MIT',
    entry_points={
        'console_scripts': [
            'trajectory_node = ackermann_trajectory_controller.trajectory_node:main',
        ],
    },
)
