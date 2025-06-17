from setuptools import setup

package_name = 'tkio_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Andrew Ealovega',
    maintainer_email='Andrew@Ealovega.dev',
    description='TODO: Package description',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tkio_ros = tkio_ros:main',
            'dead_reckon_with_joints = tkio_ros.dead_reckon_with_joints:main',
            'serial_odom_publisher = tkio_ros.serial_odom_publisher:main',
        ],
    },
)
