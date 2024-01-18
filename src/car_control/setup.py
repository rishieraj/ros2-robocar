from setuptools import setup

package_name = 'car_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rishie',
    maintainer_email='rraj27@umd.edu',
    description='Publisher/Subscriber node for mobile_car',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subscriber_node = car_control.subscriber_node:main',
            'publisher_node = car_control.publisher_node:main',
        ],
    },
)
