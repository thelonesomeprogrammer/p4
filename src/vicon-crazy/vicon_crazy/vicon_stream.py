import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from scipy.spatial.transform import Rotation as R

import time
import math as m
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
    
    def cmdpacket(self):
        return struct.pack('<fff',self.setpoint_pos[0],self.setpoint_pos[1],self.setpoint_pos[2])

class ViconPositionNode(Node):
    def __init__(self):
        super().__init__('vicon_position_node')

        self.lunched = False

        # Logging setup
        self._min_interval = rclpy.duration.Duration(seconds=1.0)
        self._last_log_times: dict[str, rclpy.time.Time] = {}
        self._log_lock = threading.Lock()

        # Subscribers, publishers and threads
        self.pos_sub = self.create_subscription(PoseStamped, '/vicon/Group466CF/Group466CF/pose',self.vicon_callback, 20)
        self.control_sub = self.create_subscription(Point, 'cf_command', self.cf_command_received, 20)
        self.appchannel_pub = self.create_publisher(Pose, 'cf_appchannel', 20)
        self.threads = [threading.Thread(target=self.rclThread),threading.Thread(target=self.CFThread)]

        # self.lasttime = 0

        # self.lastpos = [0.0, 0.0, 0.0]
        self.firstSetpoint = True
        self.setpointDistance: list[float] = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.viapointQueue: list[list[float], list[float]] = []

        self.pos = [0,0,0]
        self.dataPacket = DataPacket()
        self.exit = False
        self.last_att = [0.0, 0.0, 0.0]
        self.cf_init()

    def cf_init(self):
        self.cb_received = threading.Event()

        logging.basicConfig(level=logging.ERROR)
        cflib.crtp.init_drivers()

        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.open_link('radio://0/80/2M')
        self.get_logger().info("Link opened!")

        self.cf.appchannel.packet_received.add_callback(self.appchannel_callback)

        self.channel = Appchannel(self.cf)
        self.loc = Localization(self.cf)

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
        print(f"msg: {msg.x, msg.y, msg.z}")
        if msg.x == 0.006969 and msg.y == 0.006969 and msg.z == 1.006969:
            self.lunched = True
            return
        self.dataPacket.setpoint_pos = [msg.x,msg.y,msg.z]

    def flip_guard(self, state_att):
        if abs(state_att[0]-self.last_att[0]) < 20 or abs(state_att[1]-self.last_att[1]) < 20 or abs(state_att[2]-self.last_att[2]) < 20:
            self.last_att = state_att
        return self.last_att
    
    def vicon_callback(self, msg: PoseStamped):
        pos = msg.pose.position
        rot = msg.pose.orientation

        self.state_pos = [pos.x, pos.y, pos.z]  # forward is x, left is y, up is z
        self.state_quat = [rot.x, rot.y, rot.z, rot.w]

        rpy = R.from_quat(self.state_quat).as_euler('xyz', degrees=True)
        roll, pitch, yaw = rpy

        state_att = (roll, pitch, yaw)

        if self.firstSetpoint:
            self.dataPacket.setpoint_pos = [pos.x, pos.y, pos.z+0.5]
            self.last_att = [state_att[0], state_att[1], state_att[2]]
            self.firstSetpoint = False

        state_att = self.flip_guard(state_att)
        self.loc.send_extpose(self.state_pos, self.state_quat)

        # tempPacket = DataPacket(state_pos = state_pos, state_vel = state_vel, state_att = state_att, setpoint_pos = self.cmd_pos)

        if self._should_log("viconCB"):
                self.get_logger().info(
                    f"viconCB sent: \n" 
                    f"Position: {self.state_pos} m | "
                    f"Orientation: {state_att} | "
                    )

        # self.dataPacket = tempPacket

    def setpointDistanceUpdate(self):
        self.setpointDistance[0] = self.dataPacket.setpoint_pos[0]-self.state_pos[0]
        self.setpointDistance[1] = self.dataPacket.setpoint_pos[1]-self.state_pos[1]
        self.setpointDistance[2] = self.dataPacket.setpoint_pos[2]-self.state_pos[2]

        self.setpointDistance[3] = m.sqrt((self.state_pos[0]-self.dataPacket.setpoint_pos[0])**2
                                         +(self.state_pos[1]-self.dataPacket.setpoint_pos[1])**2
                                         +(self.state_pos[2]-self.dataPacket.setpoint_pos[2])**2)
        
        if len(self.viapointQueue) > 0:
            self.setpointDistance[4] = m.sqrt((self.state_pos[0]-self.viapointQueue[0][0])**2
                                            +(self.state_pos[1]-self.viapointQueue[0][1])**2
                                            +(self.state_pos[2]-self.viapointQueue[0][2])**2)

    def setpoint_sender(self):
        self.setpointDistanceUpdate()

        viapointAmount: int = m.floor(self.setpointDistance[3]*10)

        for i in range(viapointAmount-1,-1,-1):
            viapoint: list[float] = [self.dataPacket.setpoint_pos[0]-i/viapointAmount*self.setpointDistance[0]
                                    ,self.dataPacket.setpoint_pos[1]-i/viapointAmount*self.setpointDistance[1]
                                    ,self.dataPacket.setpoint_pos[2]-i/viapointAmount*self.setpointDistance[2]]
            self.viapointQueue.append(viapoint)
        print(self.viapointQueue)
        self.setpointDistanceUpdate()

        while(self.setpointDistance[3] >= 0.01):
            print("In setpoint loop")
            while(self.setpointDistance[4] >= 0.01):
                print("In viapoint loop")
                if self._should_log("CFThread"):
                    self.channel.send_packet(struct.pack('<fff',self.viapointQueue[0][0],self.viapointQueue[0][1],self.viapointQueue[0][2]))
                    self.get_logger().info(
                        f"CFThread sent:\n "
                        f"Setpoint_XPos: {self.viapointQueue[0][0]} |"
                        f"Setpoint_YPos: {self.viapointQueue[0][1]} |"
                        f"Setpoint_ZPos: {self.viapointQueue[0][2]} |"
                        )
                self.setpointDistanceUpdate()
            self.viapointQueue.pop(0)

    def CFThread(self):
        # i = 0
        # oldcmd = self.dataPacket.setpoint_pos
        while not self.exit:
            # time.sleep(0.01)
            if self.channel is None or self.dataPacket is None or self.lunched is False:
                continue
            # if oldcmd is self.dataPacket.setpoint_pos and i < 50:
            #     i += 1
            #     continue
            # i = 0
            # oldcmd = self.dataPacket.setpoint_pos

            self.setpoint_sender()
                
    def rclThread(self):
        rclpy.spin(self)
        self.exit = True
        rclpy.shutdown()

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
