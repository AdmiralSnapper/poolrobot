from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    #Get realsense package in /opt/ros/humble/share/
    realsense_pkg_dir = get_package_share_directory('realsense2_camera')
    #Get the path to the launch file in the realsense package
    realsense_launch_path = os.path.join(realsense_pkg_dir, 'launch', 'rs_launch.py')

    #Get directory of poolrobot package
    poolrobot_pkg_dir = get_package_share_directory('poolrobot')
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

        #Run my algorithm to count tf.
        Node(
            package = 'poolrobot',
            executable = 'cal_checker_node',
            name = 'cal_checker'
        ),

        #Run algorithm to send raw image over socket client
        Node(
            package = 'poolrobot',
            executable = 'socket_client_node',
            name = 'socket_client'
        )
    ])
