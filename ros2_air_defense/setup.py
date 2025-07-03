from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ros2_air_defense'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
        
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        
        # Install script files
        (os.path.join('lib', package_name), glob('scripts/*.py')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'sensor_msgs',
        'geometry_msgs',
        'action_msgs',
        'builtin_interfaces',
        'cv_bridge',
        'numpy',
        'opencv-python',
        'ultralytics',  # For YOLO
    ],
    zip_safe=True,
    maintainer='Air Defense Team',
    maintainer_email='team@airdefense.com',
    description='ROS2-based Air Defense System for autonomous target engagement',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_bridge_node = ros2_air_defense.scripts.vision_bridge_node:main',
            'coordinate_transformer_node = ros2_air_defense.scripts.coordinate_transformer_node:main',
            'motor_controller_node = ros2_air_defense.scripts.motor_controller_node:main',
            'firing_controller_node = ros2_air_defense.scripts.firing_controller_node:main',
            'air_defense_orchestrator_node = ros2_air_defense.scripts.air_defense_orchestrator_node:main',
        ],
    },
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: System :: Hardware :: Hardware Drivers',
    ],
    python_requires='>=3.8',
) 