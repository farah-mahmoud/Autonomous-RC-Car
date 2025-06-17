from setuptools import find_packages, setup

package_name = 'teleop_buffer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
    'teleop_buffer',
    'hardware',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='farah',
    maintainer_email='farahmahmoud031@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        #'teleop_serial = teleop_buffer.teleop_serial:main',
        'new_teleop_serial = teleop_buffer.new_teleop_serial:main',
        #'toggle = teleop_buffer.toggle:main',
        #'continous = teleop_buffer.continous:main',
        'submit = teleop_buffer.submit:main',
        ],
    },
)
