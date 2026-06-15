import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage 
from geometry_msgs.msg import PoseArray,Pose
import tf_transformations
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class TagPositionNode(Node):
    
    def __init__(self):
        super().__init__('tag_position_node')

        #Create subscriber looking for /tf, every time the topic is published tf_callback is called
        self.tf = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10,
        )

        qos_profile = QoSProfile(
            depth = 1,
            history = HistoryPolicy.KEEP_LAST,
            durability = DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pose_publisher = self.create_publisher(
            PoseArray,
            '/my_calculated_poses',
            qos_profile
        )

        self.messageCounter = 0

        #Final Global Tag Coordinates
        self.relativeTagFrames = [] 
        #ID of tag being used as a reference frame
        self.id_n = 0

    def tf_callback(self,msg):
        
        #Only read every 100 messages
        self.messageCounter += 1
        if(self.messageCounter % 100 != 0):
            return

        #Get tag detections
        rawTagFrames = []
        for t in msg.transforms:
            # Check if the frame represents a tag
            if ':' in t.child_frame_id:
                # Create a dictionary for this specific tag
                frame = {
                    'id': int(t.child_frame_id.split(':')[-1]),
                    'x': t.transform.translation.x,
                    'y': t.transform.translation.y,
                    'z': t.transform.translation.z,
                    'qx': t.transform.rotation.x,
                    'qy': t.transform.rotation.y,
                    'qz': t.transform.rotation.z,
                    'qw': t.transform.rotation.w
                }
                rawTagFrames.append(frame)
                

        #If tag list is empty:
        if len(self.relativeTagFrames) == 0:
            #No tags detected
            if(len(rawTagFrames) == 0):
                self.get_logger().info("NO TAGS DETECTED")
            #One tag detected, this can be the initial coordinate.
            elif(len(rawTagFrames) == 1):
                self.id_n = rawTagFrames[0]['id']
                self.get_logger().info(f"INITIAL TAG FOUND: Tag {self.id_n}")
                pose = {
                    'id': self.id_n,
                    # 'pose': np.eye(4) #Set pose as origin (Identity T matrix).
                    'pose': np.array([
                        [1,0,0,0],
                        [0,0,-1,0],
                        [0,1,0,0],
                        [0,0,0,1],
                    ])
                }
                self.relativeTagFrames.append(pose)
            #Too many tags detected
            else:
                self.get_logger().info("TOO MANY TAGS DETECTED, NEEDS ONE ONLY")
        


        #At least one pose has been stored.      
        if len(self.relativeTagFrames) > 0:
            #If 2 tags detected and one contains tag with id_n, and check that the tag is not already matching. 
            if len(rawTagFrames) == 2 and ({tag['id'] for tag in rawTagFrames} & {pose['id'] for pose in self.relativeTagFrames}) == {self.id_n}:
                
                #Initialise second tag ID
                id_np1 = 0
                #Initialise transformations from camera to tags
                T_n = np.zeros(4)
                T_np1 = np.zeros(4)
                
                #Find second tag ID. The one that isn't tagID_n
                for frame in rawTagFrames:
                    if not frame['id'] == self.id_n:
                        id_np1 = frame['id']

                self.get_logger().info(f"TWO TAGS DETECTED, CALCULATING RELATIVE POSE FROM {self.id_n} to {id_np1}")
                
                for frame in rawTagFrames:
                    #Create 4x4 transformation matrix T
                    T = tf_transformations.quaternion_matrix([frame['qx'], frame['qy'], frame['qz'], frame['qw']])
                    T[0:3, 3] = [frame['x'], frame['y'], frame['z']]

                    #Store transformation from camera to tag
                    if frame['id'] == self.id_n:
                        T_n = T
                    else:
                        T_np1 = T        
                        
                #Find transformation between tag n frame to tag n+1 frame
                T_n_np1 = np.dot(np.linalg.inv(T_n),T_np1)

                #Create and append a new pose by transforming the previous.
                newRelativeFrame = {
                    'id': id_np1,
                    'pose': np.dot(self.relativeTagFrames[-1]['pose'], T_n_np1)
                } 
                self.relativeTagFrames.append(newRelativeFrame)
                self.get_logger().info(f"ADDED POSE OF TAG: {id_np1}")

                #Set new tag n
                self.id_n = id_np1

                #Publish Poses as array to be read in RViz
                self.publish_poses_as_array()
                #Publish Poses as YAML file to be read by robot.
                self.publish_frames_to_yaml()


            else:
                self.get_logger().info("LINE UP PREVIOUS TAG WITH NEXT UNKNOWN TAG")
                self.get_logger().info(f"previously logged tags detected: {({tag['id'] for tag in rawTagFrames} & {pose['id'] for pose in self.relativeTagFrames})}")

    #Publish Poses as YAML file to be read by robot or testing.
    def publish_frames_to_yaml(self):
        
        tag_data = {}

        for pose in self.relativeTagFrames:
           id = int(pose['id'])
           T = pose['pose']

           #Flatten matrix into a 16 element python list
           Tflat = T.flatten().tolist()

           Tflat = [round(val, 5) for val in Tflat]

           #Map the ID directly to the matrix string 
           tag_data[f"tag_{id}"] = Tflat
        
        yaml_data = {'relative_tag_poses': tag_data}


        # Find path for YAML file: 
        try:
            package_dir = get_package_share_directory('poolrobot')
            # Saves directly into the package's shared configuration space
            yaml_path = os.path.join(package_dir, 'config', 'at_map.yaml')
            
            # Ensure the config directory exists in the installation folder
            os.makedirs(os.path.dirname(yaml_path), exist_ok=True)
        except Exception as e:
            # Fallback to local execution directory if running outside a ROS 2 launch environment
            self.get_logger().warn(f"Could not find package directory, saving locally. Error: {e}")
            yaml_path = 'at_map.yaml'

        # Write out to the YAML file
        with open(yaml_path, 'w') as f:
            # default_flow_style=None forces lists onto a single horizontal line
            yaml.dump(yaml_data, f, default_flow_style=None, sort_keys=False)
            
        self.get_logger().info(f"Successfully saved AprilTag map to: {yaml_path}")

    #Publish Poses to be Displayed in RViz.
    def publish_poses_as_array(self):
        array_msg = PoseArray()

        #Structural Metatdata
        array_msg.header.stamp = self.get_clock().now().to_msg()
        array_msg.header.frame_id = 'camera_link'

        for frame in self.relativeTagFrames:
            T = frame['pose']
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
        self.pose_publisher.publish(array_msg)
        



#MAIN FUNCTION ========================================================
def main(args = None):
        rclpy.init(args = args)
        node = TagPositionNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()



        
