from setuptools import find_packages, setup

package_name = 'gelsight_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander Minor',
    maintainer_email='alexander@minor.pw',
    description='ROS 2 driver package for GelSight tactile sensors.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gelsight_driver_node = gelsight_driver.gelsight_driver_node:main',
            'gelsight_driver = gelsight_driver.gelsight_driver_node:main',
        ],
    },
)
