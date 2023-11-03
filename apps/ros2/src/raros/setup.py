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
            'simple_publisher = raros.simple_publisher:main',
            'simple_subscriber = raros.simple_subscriber:main',
            'led_controller = raros.led_controller:main',
            'led_api_endpoint = raros.led_api_endpoint:main',
            'magnet = raros.magnet:main',
        ],
    },
)
