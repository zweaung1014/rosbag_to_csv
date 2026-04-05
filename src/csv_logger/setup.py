from setuptools import find_packages, setup

package_name = 'csv_logger'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS2 node that subscribes to multiple topics and logs messages to CSV files',
    license='MIT',
    entry_points={
        'console_scripts': [
            'csv_logger_node = csv_logger.csv_logger_node:main',
        ],
    },
)
