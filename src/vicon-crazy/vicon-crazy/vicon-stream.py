import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, Point, Pose
from scipy.spatial.transform import Rotation as R

import time
import logging
import struct
import threading
from typing import List
import cflib
from cflib.crazyflie import Crazyflie, Localization
from cflib.crazyflie.appchannel import Appchannel


class DataPacket:
    state_pos: List[float] = []
    state_vel: List[float] = []
    state_att: List[float] = []
    setpoint_pos: List[float] = []

    def __init__(self, state_pos = (0.0, 0.0, 0.0), state_vel = (0.0, 0.0, 0.0), state_att = (0.0, 0.0, 0.0), setpoint_pos = (0.0, 0.0, 0.0)):
        self.state_pos.extend(state_pos)
        self.state_vel.extend(state_vel)
        self.state_att.extend(state_att)
        self.setpoint_pos.extend(setpoint_pos)

    def returnType(self,msg_type):
        data = None
        match msg_type:
            case 1:
                data = struct.pack('<b', False)
                data += b''.join([struct.pack('<f', val) for val in self.state_pos + self.state_vel])
            case 2:
                data = struct.pack('<b', True)
                data += b''.join([struct.pack('<f', val) for val in self.state_att + self.setpoint_pos])
        return data

class ViconPositionNode(Node):
    def __init__(self):
        super().__init__('vicon_position_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        self.pos_pub = self.create_publisher(Pose, 'cf_pos', 20)
        self.control_sub = self.create_subscription(Point, 'cf_command', self.cf_command_resived, 20)
        self.threads = [threading.Thread(target=self.rlcThread),threading.Thread(target=self.CFThread)]
        self.lasttime = rclpy.time.Time()
        self.lastpos = [0.0, 0.0, 0.0]
        self.cmd_pos = [0.0, 0.0, 0.0]
        self.dataPacket = DataPacket()
        self.exit = False
        self.cf_init()

    def cf_init(self):
        self.cb_received = threading.Event()

        logging.basicConfig(level=logging.ERROR)
        cflib.crtp.init_drivers()

        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.open_link('radio://0/80/2M')
        self.get_logger().info("Link opened!")

        self.cf.appchannel.packet_received.add_callback(self.appchannel_callback)
        self.cb_received.set()
        self.get_logger().info("Callback added and set")

        self.channel = Appchannel(self.cf)
        self.loc = Localization(self.cf)

    def runThreads(self):
        self.threads[0].start()
        self.threads[1].start()

    def joinThreads(self):
        self.threads[0].join()
        self.threads[1].join()

    def zero_pos(self):
        self.loc.send_extpose([0,0,0],[0,0,0,1])

    def cf_command_resived(self, msg):
        self.cmd_pos = [msg.x,msg.y,msg.z]
        

    def timer_callback(self):
        time = rclpy.time.Time()
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'world', 'Group466CF', time
            )
        except Exception:
            return  # Wait for valid transform
        pos = trans.transform.translation
        rot = trans.transform.rotation
        quat = [rot.x, rot.y, rot.z, rot.w]
        rpy = R.from_quat(quat).as_euler('xyz', degrees=True)
        roll, pitch, yaw = rpy

        dt = time - self.lasttime
        self.lasttime = time

        state_pos = (pos.x, pos.y, pos.z)
        state_vel = ((pos.x-self.lastpos[0])/dt, (pos.y-self.lastpos[1])/dt, (pos.z-self.lastpos[2])/dt)
        state_att = (roll, pitch, yaw)

        tempPacket = DataPacket(state_pos = state_pos, state_vel = state_vel, state_att = state_att, setpoint_pos = self.cmd_pos)

        self.dataPacket = tempPacket
        
        msg = Pose(position = Point(x=pos.x, y=pos.y, z=pos.z), orientation = rot)

        self.pos_pub.publish(msg)

        self.get_logger().info(
            f"x: {pos.x:.3f}, y: {pos.y:.3f}, z: {pos.z:.3f} | "
            f"roll: {roll:.1f}°, pitch: {pitch:.1f}°, yaw: {yaw:.1f}°"
        )
    
    def rlcThread(self):
        rclpy.spin(self)
        self.exit = True
        rclpy.shutdown()

    def CFThread(self):
        while not self.exit:
            time.sleep(1)
            if self.channel is None or self.dataPacket is None:
                return
              
            self.channel.send_packet(self.dataPacket.returnType(1))
            self.channel.send_packet(self.dataPacket.returnType(2))

    def appchannel_callback(self,data):
        self.get_logger().info(
            f"XPosition: {struct.unpack('<f', data[0:4])[0]} m | "
            f"YPosition: {struct.unpack('<f', data[4:8])[0]} m | "
            f"ZPosition: {struct.unpack('<f', data[8:12])[0]} m \n"

            f"Battery Voltage: {struct.unpack('<f', data[12:16])[0]} V | "
            f"CMD_Thrust: {struct.unpack('<f', data[16:20])[0]} Thrust | "
            f"DebugState: {struct.unpack('<b', data[20:21])[0]} 1:True, 0:False | "
        )
    
    def mainLoop(self): 
        while not self.exit:
            time.sleep(1)

        self.cf.close_link()
        self.joinThreads()
   
def app():
    rclpy.init()
    node = ViconPositionNode()
    node.runThreads()
    node.mainLoop()
    
if __name__ == '__main__':
    app()
