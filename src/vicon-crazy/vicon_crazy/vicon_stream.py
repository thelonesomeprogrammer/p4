import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
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
    state_pos: List[float]
    state_vel: List[float]
    state_att: List[float]
    setpoint_pos: List[float]

    def __init__(self, state_pos = [0.0, 0.0, 0.0], state_vel = [0.0, 0.0, 0.0], state_att = [0.0, 0.0, 0.0], setpoint_pos = [0.0, 0.0, 0.5]):
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
        super().__init__('vicon_position_node')

        # Logging setup
        self._min_interval = rclpy.duration.Duration(seconds=1.0)
        self._last_log_times: dict[str, rclpy.time.Time] = {}
        self._log_lock = threading.Lock()

        # Subscribers, publishers and threads
        self.pos_sub = self.create_subscription(PoseStamped, '/vicon/Group466CF/Group466CF/pose',self.vicon_callback, 20)
        self.control_sub = self.create_subscription(Point, 'cf_command', self.cf_command_received, 20)
        self.appchannel_pub = self.create_publisher(Pose, 'cf_appchannel', 20)
        self.threads = [threading.Thread(target=self.rclThread),threading.Thread(target=self.CFThread)]

        self.lasttime = 0

        self.lastpos = [0.0, 0.0, 0.0]
        self.cmd_pos = [0.0, 0.0, 0.5]

        self.pos = [0,0,0]
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
        self.loc.send_extpose([0,0,0], [0,0,0,1])

    def runThreads(self):
        self.threads[0].start()
        for i in range(20):
            time.sleep(0.1)
        self.threads[1].start()

    def joinThreads(self):
        self.threads[0].join()
        self.threads[1].join()

    def _should_log(self, site_key: str) -> bool:
        now = self.get_clock().now()
        with self._log_lock:
            last = self._last_log_times.get(site_key)
            if last is None or (now - last) >= self._min_interval:
                self._last_log_times[site_key] = now
                return True
            return False

    def cf_command_received(self, msg: Point):
        self.cmd_pos = [msg.x,msg.y,msg.z]
        
    def vicon_callback(self, msg: PoseStamped):
        pos = msg.pose.position
        rot = msg.pose.orientation

        self.lastpos = [self.pos[0], self.pos[1], self.pos[2]]
        self.pos = [pos.x, pos.y, pos.z]

        time = float(float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec)/10**9)
        dt = time - self.lasttime
        self.lasttime = time

        state_pos = (pos.x, pos.y, pos.z)  # forward is x, left is y, up is z
        state_vel = ((pos.x-self.lastpos[0])/dt, (pos.y-self.lastpos[1])/dt, (pos.z-self.lastpos[2])/dt)

        quat = [rot.x, rot.y, rot.z, rot.w]
        rpy = R.from_quat(quat).as_euler('xyz', degrees=True)
        roll, pitch, yaw = rpy

        state_att = (roll, pitch, yaw)
        self.loc.send_extpose(state_pos, quat)
        tempPacket = DataPacket(state_pos = state_pos, state_vel = state_vel, state_att = state_att, setpoint_pos = self.cmd_pos)

        # if self._should_log("viconCB"):
        #         self.get_logger().info(
        #             f"viconCB sent: \n" 
        #             f"Position: {state_pos} m | "
        #             f"Orientation: {state_att} | "
        #             )

        self.dataPacket = tempPacket
    
    def rclThread(self):
        rclpy.spin(self)
        self.exit = True
        rclpy.shutdown()

    def CFThread(self):
        while not self.exit:
            time.sleep(0.01)
            if self.channel is None or self.dataPacket is None:
                return
            self.channel.send_packet(self.dataPacket.returnType(1))
            self.channel.send_packet(self.dataPacket.returnType(2))

            if self._should_log("CFThread"):
                self.get_logger().info(
                    f"CFThread sent:\n "
                    f"State_Pos: {self.dataPacket.state_pos} |"
                    f"State_Vel: {self.dataPacket.state_vel} |" 
                    f"State_Att: {self.dataPacket.state_att} |"
                    f"Setpoint_Pos: {self.dataPacket.setpoint_pos} |"
                    )

    def appchannel_callback(self,data):
        position = [struct.unpack('<f', data[0:4])[0],struct.unpack('<f', data[4:8])[0],struct.unpack('<f', data[8:12])[0]]
        attitude = [struct.unpack('<f', data[12:16])[0],struct.unpack('<f', data[16:20])[0],struct.unpack('<f', data[20:24])[0]]
        # compare = [struct.unpack('<i', data[24:28])[0]]
        orientation = R.from_euler('xyz', attitude, degrees=True).as_quat()

        self.appchannel_pub.publish(Pose(position=Point(x=position[0], y=position[1], z=position[2]), 
                                         orientation=Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])))

        if self._should_log("Appchannel"):
            self.get_logger().info(
                f"Appchannel received: \n"
                f"Position: {position} m | "
                f"Orientation: {attitude} | "
                # f"Compare: {compare} | "

                # f"Battery Voltage: {struct.unpack('<f', data[12:16])[0]} V | "
                # f"CMD_Thrust: {struct.unpack('<f', data[16:20])[0]} Thrust | "
                # f"DebugState: {struct.unpack('<b', data[20:21])[0]} | "
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