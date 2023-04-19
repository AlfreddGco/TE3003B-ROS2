import os
from glob import glob
from setuptools import setup

package_name = 'puzzlebot_gazebo'

setup(
    name=package_name,
    version='0.1',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob(os.path.join('launch', '*.launch'))),
        ('share/' + package_name + '/urdf',
            glob(os.path.join('urdf', '*.xacro'))),
        ('share/' + package_name + '/urdf',
            glob(os.path.join('urdf', '*.gazebo'))),
        ('share/' + package_name + '/urdf',
            glob(os.path.join('urdf', '*.urdf'))),
        ('share/' + package_name + '/meshes',
            glob(os.path.join('meshes', '*.stl'))),
        ('share/' + package_name + '/config_files',
            glob(os.path.join('config_files', '*.yaml'))),
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='ROS 2 Developer',
    author_email='ros2@ros.com',
    maintainer='ROS 2 Developer',
    maintainer_email='ros2@ros.com',
    keywords=['puzzlebot', 'gazebo'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: TODO',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Puzzlebot package with gazebo resources (urdf).',
    license='TODO',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'tf_map = ' + package_name + '.tf_map:main'
        ],
    },
)

