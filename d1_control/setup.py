from setuptools import find_packages, setup

package_name = 'd1_control'

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
    maintainer='ezeng',
    maintainer_email='ezeng@caltech.edu',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = d1_control.controller:main',
            'estimator = d1_control.estimator:main',
        ],
    },
)
