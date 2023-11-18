import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'raros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francisco Metzger',
    maintainer_email='francisco.metzger@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_api = raros.action_api:main',
            'magnet = raros.magnet:main',
            'buzzer = raros.buzzer:main',
            'color_sensor = raros.color_sensor:main',
        ],
    },
)

