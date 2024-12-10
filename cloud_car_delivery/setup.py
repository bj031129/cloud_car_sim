from setuptools import setup
import os
from glob import glob

package_name = 'cloud_car_delivery'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='bai',
    maintainer_email='3024895118@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delivery_node = cloud_car_delivery.delivery_node:main',
            'http_client_node = cloud_car_delivery.http_client_node:main',
        ],
    },
)
