from setuptools import find_packages, setup
import glob

package_name = 'robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'config/tags.yaml',
                                   'config/rviz_config.rviz']),
        ('share/' + package_name + '/launch', glob.glob('launch/*launch.*')),
        ('share/' + package_name + '/config', glob.glob('config/*')),
        ('share/' + package_name + '/images', glob.glob('dicom/top_carm/*')),
        ('share/' + package_name + '/images', glob.glob('dicom/side_carm/*')),
        ('share/' + package_name + '/images', glob.glob('png/side_carm/*')),
        ('share/' + package_name + '/images', glob.glob('png/side_carm/*'))


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kassidy Shedd',
    maintainer_email='kass92800@gmail.com',
    description='Handles all camera calibration, image processing, trajectory calculation, and robot movement',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_calibration_node = robot.camera_calibration:main',
            'move_robot_node = robot.move_robot:main'
        ],
    },
)
