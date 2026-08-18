from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    #Get realsense package in /opt/ros/humble/share/
    realsense_pkg_dir = get_package_share_directory('realsense2_camera')
    #Get the path to the launch file in the realsense package
    realsense_launch_path = os.path.join(realsense_pkg_dir, 'launch', 'rs_launch.py')

    #Get directory of poolrobot package
    poolrobot_pkg_dir = get_package_share_directory('poolrobot')
    #Join the directory of the tag YAML file and the poolrobot package
    at_map_path = os.path.join(poolrobot_pkg_dir, 'config', 'at_map.yaml')
    #Join the directory of the calibration YAML file and the poolrobot package
    calibration_path = os.path.join(poolrobot_pkg_dir, 'config', 'air_calibration.yaml')
    
    
    included_realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
    )
    
    return LaunchDescription([
        
        #Run the realsense launch file
        included_realsense_launch,

        #Run the apriltag processor
        Node(
            package = 'apriltag_ros',
            executable = 'apriltag_node',
            name = 'apriltag_processor',
            remappings = [
                ('image_rect' , '/my_image_rect'),
                ('camera_info', '/my_camera_info'),
            ],
            parameters = [{
                'detector.threads': 1,
                'size': 0.225,
            }],
            arguments = ['--ros-args', '--log-level', 'error'],
        ),

        #Run the calibration publisher to publish the calibration data
        Node(
            package = 'poolrobot',
            executable = 'calibrator_node',
            name = 'calibrator',
            parameters=[{'calyaml_path': calibration_path}]
        ),

        #Run my algorithm to localise camera.
        Node(
            package = 'poolrobot',
            executable = 'cam_localisation_node',
            name = 'cam_localisation',
            parameters=[{'map_path': at_map_path}]
        ),

        #Run the raw image socket node to stream images to the PC
        Node(
            package = 'poolrobot',
            executable = 'socket_client_node',
            name = 'socket_client',
        )
    ])