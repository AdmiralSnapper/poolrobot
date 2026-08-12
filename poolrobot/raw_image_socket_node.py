#!/usr/bin/env python3
# File: ros2_image_socket_sender.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import socket
import struct

class ImageSocketSender(Node):
    def __init__(self, target_ip='192.168.1.100', target_port=5000):
        super().__init__('image_socket_sender')
        self.bridge = CvBridge()
        self.target_ip = target_ip
        self.target_port = target_port

        # Connect TCP socket to receiver PC
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.get_logger().info(f"Connecting to PC at {self.target_ip}:{self.target_port}...")
        self.sock.connect((self.target_ip, self.target_port))
        self.get_logger().info("Connected to PC!")

        # Subscribe to RealSense image topic
        self.sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image to OpenCV BGR
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Compress image to JPEG (80% quality balances quality and speed)
            success, encoded_img = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not success:
                return

            data = encoded_img.tobytes()
            # Send 4-byte size header + JPEG payload
            header = struct.pack('>I', len(data))
            self.sock.sendall(header + data)

        except Exception as e:
            self.get_logger().error(f"Failed to stream frame over socket: {e}")

    def destroy_node(self):
        try:
            self.sock.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSocketSender(target_ip='172.16.24.125', target_port=5000)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
