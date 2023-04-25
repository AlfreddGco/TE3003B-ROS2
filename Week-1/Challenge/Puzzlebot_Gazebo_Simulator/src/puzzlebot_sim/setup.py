import os
from setuptools import setup
from glob import glob

package_name = 'puzzlebot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['simulation_config.rviz']),
        ('share/' + package_name + '/launch',
            glob(os.path.join('launch', '*.launch*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alfredo Garcia',
    maintainer_email='alfredd.gco@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_calculation = ' + package_name + '.pose_calculation:main'
        ],
    },
)

