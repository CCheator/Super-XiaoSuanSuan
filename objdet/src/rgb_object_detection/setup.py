from setuptools import setup

package_name = 'rgb_object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    description='ROS2 Jazzy RGB Object Detection',
    entry_points={
        'console_scripts': [
            'image_publisher = rgb_object_detection.camera_node:main',
            'detector_node = rgb_object_detection.detector_node:main',
        ],
    },
)
