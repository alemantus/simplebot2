from setuptools import setup

package_name = 'simplebot_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Install launch files
        ('share/' + package_name + '/launch', ['launch/display.launch.py',
                                               'launch/gui.launch.py',
                                               'launch/simplebot_bringup.launch.py']),
        # Install URDF/Xacro
        ('share/' + package_name + '/description', [
            'description/combined_description.xacro',
            'description/simplebot_description.xacro',
            'description/robot.urdf']),
        # Install meshes
        ('share/' + package_name + '/meshes', [
            # List your mesh files or glob
        ]),
        # Install RViz configs
        ('share/' + package_name + '/rviz', [
            'rviz/urdf_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Alexander',
    maintainer='Alexander',
    description='Simplebot description package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'mecanum_joint_state_publisher = simplebot_description.mecanum_joint_state_publisher:main',
        ],
    },
)
