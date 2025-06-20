import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'flying_ndogman'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    package_data={
        'flying_ndogman': ['models/*.pt'],  # Add this
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='farqil',
    maintainer_email='farrel.aqilla.nov@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = flying_ndogman.opencam:main',
            'impros = flying_ndogman.impros:main',
            'gimbal_lock = flying_ndogman.gimbal_locking:main',
            'dropping_mechanism = flying_ndogman.uav_system:main',
            'test = flying_ndogman.test:main',
        ],
    },
)
