
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


class TagLocalisationNode(Node):
    
    def __init__(self):
        super().__init__('cam_localisation_node')

        #Create subscriber looking for /tf, every time the topic is published tf_callback is called
        self.tf = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10,
        )

        #Create publisher to publish map poses.
        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL # <-- THIS IS THE CRUCIAL LINE
        )
        self.tag_pose_publisher = self.create_publisher(
            PoseArray,
            '/tag_poses',
            qos_profile
        )

        #Create publisher to publish the calculated camera pose for RViz.
        self.cam_pose_publisher = self.create_publisher(
            PoseStamped,
            '/camera_pose',
            10
        )

        #Create publisher to publish the path of camera pose for RViz.
        self.cam_path_publisher = self.create_publisher(
            Path,
            '/camera_path',
            10
        )
        self.cam_path = Path()

        #Read YAML poses from file in config/
        self.yamlRelativeTagFrames = self.initialiseTagFrames()
        #Publish YAML poses to RViz
        self.publishTagPoses()

        #Initialise csv folder for data logging.
        self.knownTagIDs = [frame['id'] for frame in self.yamlRelativeTagFrames]
        tagHeaders = ["Timestamp"] + [f"Tag {tag_id}" for tag_id in self.knownTagIDs]
        locHeaders = ["Timestamp","x","y","z","roll","pitch","yaw"]

        self.distancesCSVwriter = self.initialiseCSV('distances',tagHeaders)
        self.rotationsCSVwriter = self.initialiseCSV('rotations',tagHeaders)
        self.cameraPoseCSVwriter = self.initialiseCSV('cameraPose',locHeaders)

        #Store Start Time
        self.startTime = self.get_clock().now()

    def tf_callback(self,msg):

        #Initialise array of tag poses
        rawTagFrames = []
        distances = []
        rotations = []
        
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

                self.get_logger().info("appended tag frame")

        #Calculate distances of each pose.
        minReliability = 100
        minReliabilityID = 100
        for frame in rawTagFrames:
            T = frame['pose']

            #Calculate overall distance in meters
            x = T[0][3]
            y = T[1][3]
            z = T[2][3]
            distance = math.hypot(x, y, z)

            #Calculate overall rotation in degrees
            R = T[0:3,0:3]
            matrix_trace = np.trace(R)
            cos_theta = (matrix_trace -1.0)/2.0
            cos_theta = np.clip(cos_theta, -1.0, 1.0)
            rotation = np.degrees(np.arccos(cos_theta))
            rotation = np.abs(rotation - 180)

            #Store distances and rotations of all tags to array
            distances.append(distance)
            rotations.append(rotation)

            #Determine how reliable the tag is 
            reliability = np.cos(np.deg2rad(rotation))/(distance*distance)

            if reliability < minReliability:
                minReliability = reliability
                minReliabilityID = frame['id']
                minReliabilityPose = frame['pose']
            

        self.writeDistanceOrRotationToCSV(rawTagFrames,distances,self.distancesCSVwriter)
        self.writeDistanceOrRotationToCSV(rawTagFrames,rotations,self.rotationsCSVwriter)

        #Calculate Camera Pose
        if len(rawTagFrames) >= 1:
            #Get the detected tag ID and its matrix
            tag_id = minReliabilityID
            Ttf = minReliabilityPose
            
            #Index YAML poses by ID
            frame_lookup = {frame['id']: frame['pose'] for frame in self.yamlRelativeTagFrames}
            
            #Safely grab the matching NumPy matrix for this specific tag
            if tag_id in frame_lookup:
                Tyaml = frame_lookup[tag_id]
                
                #Perform the matrix math
                Tcam = np.dot(Tyaml, np.linalg.inv(Ttf))
                self.get_logger().info(f"Camera Pose: {Tcam}")
                
                #Publish and store camera pose.
                self.publishCameraPose(Tcam)
                self.writeCameraPoseToCSV(Tcam)
            else:
                self.get_logger().warn(f"Detected tag ID {tag_id} not found in yamlposes!")

    #Read Tag frames from at_map.yaml and store in array.
    def initialiseTagFrames(self):
        self.get_logger().info("Node started! Attempting to read poses...")
        
        # 1. Declare and get the file path string parameter
        self.declare_parameter('map_path', rclpy.Parameter.Type.STRING)
        map_path = self.get_parameter('map_path').value
        
        # This will hold your array of dictionaries
        poses = []
        
        try:
            with open(map_path, 'r') as f:
                raw_data = yaml.safe_load(f)
                
            tag_data = raw_data['relative_tag_poses']
            
            # 2. Iterate through each tag entry
            for tag_key, flat_list in tag_data.items():
                
                # Extract the number from strings like "tag_4" -> 4
                tag_id = int(tag_key.split('_')[1])
                
                # Reshape the 16-element flat list into a 4x4 NumPy matrix
                t_matrix = np.array(flat_list).reshape(4, 4)
                
                # 3. Store as a dictionary with "id" and "pose" keys
                pose_entry = {
                    'id': tag_id,
                    'pose': t_matrix
                }
                
                # Append to your array list
                poses.append(pose_entry)
                
                self.get_logger().info(f"Successfully processed Tag {tag_id} into a 4x4 Matrix.")
                
            self.get_logger().info(f"Total poses stored in array: {len(self.yamlRelativeTagFrames)}")
            self.get_logger().info(f"POSES: {self.yamlRelativeTagFrames}")

        except Exception as e:
            self.get_logger().error(f"Failed to process poses: {str(e)}")

        return poses

    #Publish Tag frames to be Displayed in RViz (Directly from at_map.yaml).
    def publishTagPoses(self):
        array_msg = PoseArray()

        #Structural Metatdata
        array_msg.header.stamp = self.get_clock().now().to_msg()
        array_msg.header.frame_id = 'camera_link'

        for pose in self.yamlRelativeTagFrames:
            T = pose['pose']
            p = Pose()

            # 1. Extract position from the matrix translation column
            p.position.x = float(T[0, 3])
            p.position.y = float(T[1, 3])
            p.position.z = float(T[2, 3])

            # 2. Extract rotation quaternion from the matrix
            q = tf_transformations.quaternion_from_matrix(T)
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            # Add this pose to our array list
            array_msg.poses.append(p)

        # 3. Publish onto your isolated topic name
        self.tag_pose_publisher.publish(array_msg)

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

    #Write tag distances or rotations to csv.
    def writeDistanceOrRotationToCSV(self, rawTagFrames,values,CSVwriter):
        
        CSVrow = ["#N/A"] * len(self.knownTagIDs)

        timestamp = self.calculateTimestamp()

        if (rawTagFrames == [] and values == []):
            CSVrow.insert(0,timestamp)
            CSVwriter.writerow(CSVrow)
            return
        
        detectedTagIDs = [frame['id'] for frame in rawTagFrames]
        
        for tagID in detectedTagIDs:
            #Check if tag pose has been stored.
            if tagID not in self.knownTagIDs:
                continue

            #Get index in known tags where detected tag is stored.
            CSVindex = self.knownTagIDs.index(tagID)
            tagIndex = detectedTagIDs.index(tagID)
            CSVrow[CSVindex] = values[tagIndex]

        CSVrow.insert(0,timestamp)
        CSVwriter.writerow(CSVrow)

    #Write camera localisation to CSV with 6dof.
    def writeCameraPoseToCSV(self, Tcam):

        # Extract Translation (Position) from the 4x4 matrix
        x = float(Tcam[0, 3])
        y = float(Tcam[1, 3])
        z = float(Tcam[2, 3])

        # 2. Extract Rotation Matrix Elements
        r00, r10, r20 = Tcam[0, 0], Tcam[1, 0], Tcam[2, 0]
        r21, r22 = Tcam[2, 1], Tcam[2, 2]
        
        # 3. Calculate Euler Angles (Pitch, Roll, Yaw)
        # Handle gimbal lock safety if pitch is close to +90 or -90 degrees
        if abs(r20) < 0.99999:
            pitch = -math.asin(r20)
            roll = math.atan2(r21, r22)
            yaw = math.atan2(r10, r00)
        else:
            # Gimbal lock occurred (Nose straight up or down)
            pitch = -math.asin(r20)
            roll = 0.0  # Force roll to zero
            yaw = math.atan2(-Tcam[0, 1], Tcam[1, 1])

        timestamp = self.calculateTimestamp()

        CSVrow = [timestamp,x,y,z,roll,pitch,yaw]
        self.cameraPoseCSVwriter.writerow(CSVrow)
        
    #Calculate current timestamp in seconds
    def calculateTimestamp(self):
        # 1. Calculate the elapsed time since startup in seconds
        currentTime = self.get_clock().now()
        elapsed_duration = currentTime - self.startTime
        # Convert nanoseconds to a clean decimal float of seconds
        time_msg = elapsed_duration.to_msg()
        timestamp = time_msg.sec + (time_msg.nanosec / 1e9)
        return timestamp

    #Publish Camera Pose relative to tags for display in RViz 
    def publishCameraPose(self,Tcam):
        #Initialize the message
        msg_out = PoseStamped()
        
        # Add header metadata (Crucial for TF and tracking)
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = 'camera_link' 
        
        # Extract Translation (Position) from the 4x4 matrix
        msg_out.pose.position.x = float(Tcam[0, 3])
        msg_out.pose.position.y = float(Tcam[1, 3])
        msg_out.pose.position.z = float(Tcam[2, 3])
        
        # Extract Rotation (Quaternion) from the 4x4 matrix
        q = tf_transformations.quaternion_from_matrix(Tcam)
        msg_out.pose.orientation.x = float(q[0])
        msg_out.pose.orientation.y = float(q[1])
        msg_out.pose.orientation.z = float(q[2])
        msg_out.pose.orientation.w = float(q[3])
        
        # Publish 
        self.cam_pose_publisher.publish(msg_out)
        self.get_logger().info(f"Published Camera Pose")


        #PUBLISH AS PATH
        # Update the overall path header frame
        self.cam_path.header = msg_out.header 

        # Append the current camera pose into our history
        self.cam_path.poses.append(msg_out)

        # Limit the trail size to 500 points to prevent memory bloat
        if len(self.cam_path.poses) > 500:
            self.cam_path.poses.pop(0)

        # Publish the history array
        self.cam_path_publisher.publish(self.cam_path)


#MAIN FUNCTION ========================================================
def main(args = None):
    rclpy.init(args = args)
    node = TagLocalisationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()