from setuptools import find_packages, setup

package_name = 'radconpy'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Patryk Filip Gryz',
    maintainer_email='pfgryz@gmail.com',
    description='Small library to work with RadCon',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={},
)
