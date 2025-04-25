import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import Bool as Rbool

import time
import struct
import threading
from typing import List


class DataPacket:
    state_pos: List[float]
    state_vel: List[float]
    state_att: List[float]
    setpoint_pos: List[float]

    def __init__(self, state_pos = [0.0, 0.0, 0.0], state_vel = [0.0, 0.0, 0.0], state_att = [0.0, 0.0, 0.0], setpoint_pos = [0.0, 0.0, 0.0]):
        self.state_pos = state_pos
        self.state_vel = state_vel
        self.state_att = state_att
        self.setpoint_pos = setpoint_pos

    def returnType(self,msg_type):
        data = None
        match msg_type:
            case 1:
                data = struct.pack('<b', False)
                data += b''.join([struct.pack('<f', (val)) for val in tuple(self.state_pos) + tuple(self.state_vel) ])
            case 2:
                data = struct.pack('<b', True)
                data += b''.join([struct.pack('<f', (val)) for val in tuple(self.state_att) + tuple(self.setpoint_pos)])
        return data

class ViconPositionNode(Node):
    def __init__(self):
        super().__init__('fake_vicon_position_node')
        self.timer = self.create_timer(0.1, self.timer_callback)  # 100 Hz
        self.pos_pub = self.create_publisher(Pose, 'cf_pos', 20)
        self.link_pub = self.create_publisher(Rbool, 'cf_link', 20)
        self.control_sub = self.create_subscription(Point, 'cf_command', self.cf_command_resived, 20)
        self.threads = [threading.Thread(target=self.rlcThread),threading.Thread(target=self.CFThread)]
        self.lasttime = 0
        self.lastpos = [0.0, 0.0, 0.0]
        self.cmd_pos = [0.0, 0.0, 0.0]
        self.dataPacket = DataPacket()
        self.exit = False


    def cf_init(self):
        self.link_pub.publish(Rbool(data = True))

    def runThreads(self):
        self.threads[0].start()
        self.threads[1].start()

    def joinThreads(self):
        self.threads[0].join()
        self.threads[1].join()

    def cf_command_resived(self, msg):
        self.cmd_pos = [msg.x,msg.y,msg.z]
        

    def timer_callback(self):
        time = rclpy.time.Time().nanoseconds
        dt = time - self.lasttime
        if dt == 0:
            dt = 1
        self.lasttime = time

        state_pos = (0, 0, 0)
        state_vel = ((0-self.lastpos[0])/dt, (0-self.lastpos[1])/dt, (0-self.lastpos[2])/dt)
        state_att = (0, 0, 0)

        tempPacket = DataPacket(state_pos = state_pos, state_vel = state_vel, state_att = state_att, setpoint_pos = self.cmd_pos)

        self.dataPacket = tempPacket
        
        msg = Pose(position = Point(), orientation = Quaternion())

        self.pos_pub.publish(msg)
    
    def rlcThread(self):
        rclpy.spin(self)
        self.exit = True
        rclpy.shutdown()

    def CFThread(self):
        while not self.exit:
            time.sleep(1)
            self.link_pub.publish(Rbool(data = True))
            self.get_logger().info(
                f"pack1: {self.dataPacket.returnType(1)}"
                f"pack2: {self.dataPacket.returnType(2)}"
            )
    
    def mainLoop(self): 
        while not self.exit:
            time.sleep(1)

        self.joinThreads()
   
def app():
    rclpy.init()
    node = ViconPositionNode()
    node.runThreads()
    node.mainLoop()
    
if __name__ == '__main__':
    app()
