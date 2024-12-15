from setuptools import find_packages, setup

package_name = 'bluerov_grasp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/grasp_launch.py']),
        ('share/' + package_name + '/config', ['config/camera_info.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akshat',
    maintainer_email='sinhaakshat14@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_detection_node = bluerov_grasp.marker_detection_node:main',
            'grasp_controller_node = bluerov_grasp.grasp_controller_node:main',
            'aruco_detector = bluerov_grasp.aruco_detector:main',
            'marker_detection_simulation = bluerov_grasp.marker_detection_simulation:main',
            'marker_detection_simulation_4_10 = bluerov_grasp.marker_detection_simulation_4_10:main',
            'aruco_detection_node = bluerov_grasp.aruco_detection_node:main', 
            'aruco_follower = bluerov_grasp.aruco_follower:main',
            'bluerov_aruco_detection = bluerov_grasp.bluerov_aruco_detection:main', 
            'laptopcamera = bluerov_grasp.laptopcamera:main',
            'bluerov2_camera_publisher = bluerov_grasp.bluerov2_camera_publisher:main',
            'manual_follower_depth_aruco = bluerov_grasp.manual_follower_depth_aruco:main',
            'bluerov_marker_follower = bluerov_grasp.bluerov_marker_follower:main',
            'pressure_node = bluerov_grasp.pressure_node:main',
            'image_viewer_node = bluerov_grasp.image_viewer_node:main',
            'gui_controller = bluerov_grasp.gui_controller:main'
        ],
    },
)
