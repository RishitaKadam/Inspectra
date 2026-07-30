import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'inspectra_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'media'), [f for f in glob('media/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'media', 'conveyor_feed'), glob('media/conveyor_feed/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='YOLOv8-based object detection for the Inspectra inspection cell.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector_node = inspectra_perception.object_detector:main',
            'test_image_publisher = inspectra_perception.test_image_publisher:main',
            'conveyor_publisher = inspectra_perception.conveyor_publisher:main',
            'pose_estimator_node = inspectra_perception.pose_estimator:main',
            'conveyor_motion_node = inspectra_perception.conveyor_motion_node:main',
        ],
    },
)
