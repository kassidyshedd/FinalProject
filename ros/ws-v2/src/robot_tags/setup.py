from setuptools import find_packages, setup

package_name = 'robot_tags'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/realsense.launch.py',
                                   'launch/rs.launch.xml',
                                   'config/tags.yaml',
                                   'launch/img.launch.xml',
                                   'launch/point.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kashedd',
    maintainer_email='kass92800@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_robot = robot_tags.move_robot:main',
            'capture_frame = robot_tags.save_frame:main',
            'find_transforms = robot_tags.find_transforms:main'
            # 'click_to_point = robot_tags.click_to_point:main',
            # 'transform_point = robot_tags.transform_point:main',
        ],
    },
)
