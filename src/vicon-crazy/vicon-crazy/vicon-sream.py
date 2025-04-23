import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

import time
import logging
import struct
import threading
import asyncio
from dataclasses import dataclass
from typing import List, Tuple
import cflib
from cflib.crazyflie import Crazyflie, Localization
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.appchannel import Appchannel


class DataPacket:
    state_pos: List[float] = []
    state_vel: List[float] = []
    state_att: List[float] = []
    setpoint_pos: List[float] = []

    def __init__(self): # TODO
        self.state_pos.extend((0.0, 0.0, 0.0))
        self.state_vel.extend((0.0, 0.0, 0.0))
        self.state_att.extend((0.0, 0.0, 0.0))
        self.setpoint_pos.extend((0.0, 0.0, 0.0))

    def returnType(self,type) -> Tuple[float, ...]:
        if type == 1:
            return (*self.state_pos, *self.state_vel)
        elif type == 2:
            return (*self.state_att, *self.setpoint_pos)
        else:
            print(f'Type Error Out Of Bounds: {type}')
            return (0.0)

class ViconPositionNode(Node):
    def __init__(self):
        super().__init__('vicon_position_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        self.threads = [threading.Thread(target=self.rlcThread),threading.Thread(target=self.CFThread)]
        self.lasttime = time.perf_counter()
        self.lastpos = [0.0, 0.0, 0.0]

        self.cb_received = threading.Event()
        URI = 'radio://0/80/2M'
        self.dataPacket = DataPacket()
        self.tempPacket = DataPacket()
        self.dataPacket.setpoint_pos = [0.0,0.0,0.5]
        self.exit = False

        logging.basicConfig(level=logging.ERROR)
        cflib.crtp.init_drivers()

        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.open_link(URI)
        print("Link opened!")

        self.cf.appchannel.packet_received.add_callback(self.appchannel_callback)
        self.cb_received.set()
        print("Callback added and set")

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
            
            self.tempPacket.state_pos = [pos.x, pos.y, pos.z]

            dt = time.perf_counter() - self.lasttime
            self.lasttime = time.perf_counter()

            self.tempPacket.state_vel = [pos.x-self.lastpos[0], pos.y-self.lastpos[1], pos.z-self.lastpos[2]]/dt
            self.tempPacket.state_att = [roll, pitch, yaw]

            self.dataPacket = self.tempPacket

            self.get_logger().info(
                f"x: {pos.x:.3f}, y: {pos.y:.3f}, z: {pos.z:.3f} | "
                f"roll: {roll:.1f}°, pitch: {pitch:.1f}°, yaw: {yaw:.1f}°"
            )
        except Exception:
            pass  # Wait for valid transform
    
    def rlcThread(self):
        rclpy.spin(self)
        self.exit = True
        rclpy.shutdown()

    def CFThread(self):
        while not self.exit:
            time.sleep(1)
            if self.channel is None or self.dataPacket is None:
                return
            data = struct.pack('<b', False)
            print(f'Data: {data}')
            data += b''.join([struct.pack('<f', val) for val in self.dataPacket.returnType(1)])
            print(f'Data: {self.dataPacket.returnType(1)}')
            self.channel.send_packet(data)
            data = struct.pack('<b', True)
            data += b''.join([struct.pack('<f', val) for val in self.dataPacket.returnType(2)])
            self.channel.send_packet(data)

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
        
        while(not exit):
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
