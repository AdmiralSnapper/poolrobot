#!/usr/bin/env python3
import json
import socket
import struct
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image

MSG_TYPE_IMAGE = 1
MSG_TYPE_CAMERA_POSE = 2
MSG_TYPE_TAG_POSES = 3


class MultiSocketSender(Node):
    def __init__(self):
        super().__init__('multi_socket_sender')

        self.declare_parameter('server_ip', '172.16.24.125')
        self.declare_parameter('server_port', 5000)
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_pose_topic', '/camera_pose')
        self.declare_parameter('tag_poses_topic', '/tag_poses')
        self.declare_parameter('jpeg_quality', 80)

        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._sock = None

        ip = self.get_parameter('server_ip').value
        port = self.get_parameter('server_port').value
        self.server_addr = (ip, port)
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.on_image, 10)
        self.create_subscription(
            PoseStamped, self.get_parameter('camera_pose_topic').value,
            self.on_camera_pose, 10)
        self.create_subscription(
            PoseArray, self.get_parameter('tag_poses_topic').value,
            self.on_tag_poses, 10)
        self.get_logger().info('Socket sender started.')

    # ---------- socket plumbing ----------
    def _connect(self):
        if self._sock is not None:
            return True
        try:
            self._sock = socket.create_connection(self.server_addr, timeout=2.0)
            self.get_logger().info(f'Connected to {self.server_addr}')
            return True
        except OSError:
            self._sock = None
            return False

    def _send(self, msg_type, payload):
        try:
            if not self._connect():
                return
            with self._lock:
                self._sock.sendall(
                    struct.pack('>BI', msg_type, len(payload)) + payload)
        except OSError as e:
            self.get_logger().warn(f'Socket error: {e}; reconnecting...')
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    # ---------- subscribers ----------
    def on_image(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, jpg = cv2.imencode(
                '.jpg', cv_img,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                self._send(MSG_TYPE_IMAGE, jpg.tobytes())
        except Exception as e:
            self.get_logger().warn(f'Image encode failed: {e}')

    def on_camera_pose(self, msg):
        data = {
            'frame_id': msg.header.frame_id,
            'stamp_ns': msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
            'position': [msg.pose.position.x,
                         msg.pose.position.y,
                         msg.pose.position.z],
            'quaternion': [msg.pose.orientation.x,
                           msg.pose.orientation.y,
                           msg.pose.orientation.z,
                           msg.pose.orientation.w],
        }
        self._send(MSG_TYPE_CAMERA_POSE, json.dumps(data).encode())

    def on_tag_poses(self, msg):
        poses = []
        for i, p in enumerate(msg.poses):
            poses.append({
                'id': i,
                'position': [p.position.x, p.position.y, p.position.z],
                'quaternion': [p.orientation.x, p.orientation.y,
                               p.orientation.z, p.orientation.w],
            })
        data = {'frame_id': msg.header.frame_id, 'poses': poses}
        self._send(MSG_TYPE_TAG_POSES, json.dumps(data).encode())


def main(args=None):
    rclpy.init(args=args)
    node = MultiSocketSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
