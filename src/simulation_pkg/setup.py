import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'simulation_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_dir={".": "."},
    package_data={"": ["*.urdf"]},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # (os.path.join(package_name), [os.path.join('myblock.urdf')])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mgiglia',
    maintainer_email='michael.a.giglia@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'SimulationNode = simulation_pkg.SimulationNode:main'
        ],
    },
)
