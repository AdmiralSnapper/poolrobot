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
                ('/image_rect' , '/camera/camera/color/image_raw'),
                ('/camera_info', '/camera/camera/color/camera_info'),
            ],
            parameters = [{
                'detector.threads': 1,
                'size': 0.05,
            }],
            arguments = ['--ros-args', '--log-level', 'error'],
        ),

        #Run the localization viewer
        Node(
            package = 'rviz2',
            executable = 'rviz2',
            name = 'rviz_display',
        ),

        #Run my algorithm to count tf.
        Node(
            package = 'poolrobot',
            executable = 'cal_checker_node',
            name = 'cal_checker'
        ),
    ])
