from setuptools import find_packages, setup

package_name = 'radconpy_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'radcon_msgs', 'radcon'],
    zip_safe=True,
    maintainer='marysia',
    maintainer_email='m.waligorska@student.uw.edu.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['publisher=radconpy_ros.publisher:main'
        ],
    },
)
