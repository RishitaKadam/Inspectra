from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'inspectra_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            os.path.join('share', package_name),
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py'),
    ),
],
        
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rk',
    maintainer_email='kadamrishita3@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'controller = inspectra_bringup.inspectra_controller:main',
        'motion_planner =           inspectra_manipulation.motion_planner:main', 
        ],
    },
)
