#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

class ViconPositionNode(Node):
    def __init__(self):
        super().__init__('vicon_position_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz

    def timer_callback(self):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'world', 'Group466CF', rclpy.time.Time()
            )
            pos = trans.transform.translation
            rot = trans.transform.rotation
            quat = [rot.x, rot.y, rot.z, rot.w]
            rpy = R.from_quat(quat).as_euler('xyz', degrees=True)
            roll, pitch, yaw = rpy

            self.get_logger().info(
                f"x: {pos.x:.3f}, y: {pos.y:.3f}, z: {pos.z:.3f} | "
                f"roll: {roll:.1f}°, pitch: {pitch:.1f}°, yaw: {yaw:.1f}°"
            )
        except Exception:
            pass  # Wait for valid transform

def main():
    rclpy.init()
    node = ViconPositionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
