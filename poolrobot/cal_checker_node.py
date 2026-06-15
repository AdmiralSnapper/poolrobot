
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage 
from geometry_msgs.msg import PoseArray,Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from ament_index_python.packages import get_package_share_directory
import tf_transformations
import numpy as np
import rclpy
import yaml
import math
import csv
import os


class CalibrationCheckerNode(Node):
    
    def __init__(self):
        super().__init__('cal_checker_node')

        #Create subscriber looking for /tf, every time the topic is published tf_callback is called
        self.tf = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10,
        )

        self.comparisonTagIDs = []

        #Initialise CSV to store relative pose between two tags.
        poseHeaders = ["Timestamp","x","y","z","qx","qy","qz","qw"]
        self.CSVwriter = self.initialiseCSV('Tag Comparison', poseHeaders)

        #Store Start Time
        self.startTime = self.get_clock().now()


    def tf_callback(self,msg):

        #Initialise array of tag poses
        rawTagFrames = []
        
        #Transform detections into array of poses.
        for t in msg.transforms:
            # Check if the frame represents a tag
            if ':' in t.child_frame_id:
                # Create a dictionary for this specific tag
                tag = {
                    'id': int(t.child_frame_id.split(':')[-1]),
                    'x': t.transform.translation.x,
                    'y': t.transform.translation.y,
                    'z': t.transform.translation.z,
                    'qx': t.transform.rotation.x,
                    'qy': t.transform.rotation.y,
                    'qz': t.transform.rotation.z,
                    'qw': t.transform.rotation.w
                }

                T = tf_transformations.quaternion_matrix([tag['qx'], tag['qy'], tag['qz'], tag['qw']])
                T[0:3, 3] = [tag['x'], tag['y'], tag['z']]
                
                #Store transformation from camera to tag (with camera as origin)
                frame = {
                    'id': tag['id'],
                    'pose': T
                }
                rawTagFrames.append(frame)


        #If two tags detected, and no initialised tags, then initialise those tags.
        if len(rawTagFrames) == 2 and self.comparisonTagIDs == []:
            self.comparisonTagIDs = [rawTagFrames[0]['id'], rawTagFrames[1]['id']]

        self.get_logger().info(f"Comparison Tag IDS: {self.comparisonTagIDs}")

        #If two tags detected, and they are the same as the initialised tags.
        if len(rawTagFrames) == 2:
            if rawTagFrames[0]['id'] == self.comparisonTagIDs[0] and rawTagFrames[1]['id'] == self.comparisonTagIDs[1]:
                T0 = rawTagFrames[0]['pose']
                T1 = rawTagFrames[1]['pose']

                #Find transformation between tags
                Trelative = np.dot(np.linalg.inv(T0),T1)

                self.writePoseToCSV(Trelative)
            



    #Initialise .csv files for writing tag detections
    def initialiseCSV(self, filename, headers):
        # Find path for CSV file: 
        try:
            package_dir = get_package_share_directory('poolrobot')
            # Saves directly into the package's shared configuration space
            csv_path = os.path.join(package_dir, 'config', f'{filename}.csv')
            
            # Ensure the config directory exists in the installation folder
            os.makedirs(os.path.dirname(csv_path), exist_ok=True)
        except Exception as e:
            # Fallback to local execution directory if running outside a ROS 2 launch environment
            self.get_logger().warn(f"Could not find package directory, saving locally. Error: {e}")
            csv_path = f'{filename}.csv'

        #Open file with path and write headers.
        file = open(csv_path, mode = "w", newline='')
        CSVwriter = csv.writer(file)
        CSVwriter.writerow(headers)
        return CSVwriter

    #Write camera pose to .csv as quaternion.
    def writePoseToCSV(self, T):
        x,y,z,qx,qy,qz,qw = self.TtoQuaternion(T)
        timestamp = self.calculateTimestamp()
        CSVrow = [timestamp, x, y, z, qx, qy, qz, qw]
        self.CSVwriter.writerow(CSVrow)

    #Calculate current timestamp in seconds
    def calculateTimestamp(self):
        # 1. Calculate the elapsed time since startup in seconds
        currentTime = self.get_clock().now()
        elapsed_duration = currentTime - self.startTime
        # Convert nanoseconds to a clean decimal float of seconds
        time_msg = elapsed_duration.to_msg()
        timestamp = time_msg.sec + (time_msg.nanosec / 1e9)
        return timestamp

    #Convert a T matrix to quaternion
    def TtoQuaternion(self,T):
        # Extract translation
        x, y, z = T[0,3], T[1,3], T[2,3]
        # Extract Rotation (Quaternion) from the 4x4 matrix
        q = tf_transformations.quaternion_from_matrix(T)
        qx = float(q[0])
        qy = float(q[1])
        qz = float(q[2])
        qw = float(q[3])
        return x,y,z,qx,qy,qz,qw
    

#MAIN FUNCTION ======================================================================
def main(args = None):
    rclpy.init(args = args)
    node = CalibrationCheckerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()