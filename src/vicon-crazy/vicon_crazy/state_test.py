import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from scipy.spatial.transform import Rotation as R

import time
import logging
import struct
import threading
from typing import List
import cflib
from cflib.crazyflie import Crazyflie, Localization
from cflib.crazyflie.appchannel import Appchannel

pose = Pose()
point = Point()
quat = Quaternion()

class ViconPositionNode(Node):
    def __init__(self):
        super().__init__('fake_position_node')
        self.pos_sub = self.create_publisher(Pose, '/vicon/Group466CF/Group466CF/pose',20)
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        r = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
        quat = r.as_quat()
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    
    def mainLoop(self):
        i = 0
        while True:
            time.sleep(0.01)
            quat = self.quaternion_from_euler(0.0, 0.0, 0.0)
            point = Point(x=i+0.0, y=i+1.0, z=i+2.0)
            pose = Pose(position=point, orientation=quat)
            self.pos_sub.publish(pose)
            if i == 20:
                print("resetting")
                i = 0
            else:
                i += 1
   
def app():
    rclpy.init()
    node = ViconPositionNode()
    node.mainLoop()
    
if __name__ == '__main__':
    app()
