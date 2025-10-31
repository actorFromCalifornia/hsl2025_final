from glob import glob
import os
from setuptools import setup

package_name = 'solution_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikita Savinov',
    maintainer_email='nikita.savinov@example.com',
    description='Bringup launch files and parameters for high-level navigation stack.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)
