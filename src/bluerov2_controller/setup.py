from setuptools import find_packages, setup

package_name = 'bluerov2_controller'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "controller = bluerov2_controller.controller:main",
            "controller_ld = bluerov2_controller.controller_ld:main",
            "depth_controller = bluerov2_controller.depth_controller:main",
            "pitch_controller = bluerov2_controller.pitch_controller:main",
            "roll_controller = bluerov2_controller.roll_controller:main",
            "yaw_controller = bluerov2_controller.yaw_controller:main",
            "input_controller = bluerov2_controller.input_controller:main",
            "input_controller_ld = bluerov2_controller.input_controller_ld:main",
            "video = bluerov2_controller.video:main",
            "manual_controller = bluerov2_controller.manual_controller:main",
            "aruco_alignment_controller = bluerov2_controller.aruco_alignment_controller:main",
            "gui_controller = bluerov2_controller.gui_controller:main",
            "aruco_yaw_controller = bluerov2_controller.aruco_yaw_controller:main",
        ],
    },
)
