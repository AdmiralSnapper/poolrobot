
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import socket
import struct
import json

MSG_TYPE_IMAGE = 1
MSG_TYPE_POSE = 2

class MultiSocketSender(Node):
    def __init__(self, target_ip='192.168.1.100', target_port=5000):
        super().__init__('multi_socket_sender')
        self.bridge = CvBridge()
        self.target_ip = target_ip
        self.target_port = target_port

        # Connect socket to PC
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.get_logger().info(f"Connecting to PC at {self.target_ip}:{self.target_port}...")
        self.sock.connect((self.target_ip, self.target_port))
        self.get_logger().info("Connected to PC!")

        # Subscribe to Image topic
        self.sub_image = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Subscribe to PoseStamped topic
        self.sub_pose = self.create_subscription(
            PoseStamped,
            '/camera_pose',
            self.pose_callback,
            10
        )

    def send_payload(self, msg_type: int, payload_bytes: bytes):
        try:
            # Header format: 1 byte msg_type + 4 byte uint32 payload size
            header = struct.pack('>BI', msg_type, len(payload_bytes))
            self.sock.sendall(header + payload_bytes)
        except Exception as e:
            self.get_logger().error(f"Failed to send socket message: {e}")

    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            success, encoded_img = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if success:
                self.send_payload(MSG_TYPE_IMAGE, encoded_img.tobytes())
        except Exception as e:
            self.get_logger().error(f"Error encoding image: {e}")

    def pose_callback(self, msg: PoseStamped):
        try:
            pose_dict = {
                'frame_id': msg.header.frame_id,
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
                'position': {
                    'x': msg.pose.position.x,
                    'y': msg.pose.position.y,
                    'z': msg.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.orientation.x,
                    'y': msg.pose.orientation.y,
                    'z': msg.pose.orientation.z,
                    'w': msg.pose.orientation.w
                }
            }
            json_bytes = json.dumps(pose_dict).encode('utf-8')
            self.send_payload(MSG_TYPE_POSE, json_bytes)
        except Exception as e:
            self.get_logger().error(f"Error serialization pose: {e}")

    def destroy_node(self):
        try:
            self.sock.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    # CHANGE TO YOUR PC/LAPTOP's IP ADDRESS:
    node = MultiSocketSender(target_ip='172.16.24.125', target_port=5000)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()