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
    p: List[float] = []
    v: List[float] = []
    # a: List[float] = [] # Removed acceleration to reduce data size
    yaw: float
    # yawRate: float # Removed yawRate to reduce data size

    def asTuple(self) -> Tuple[float, ...]:
        # melt p, v, a + yaw, yawRate into one flat tuple
        return (*self.p, *self.v, self.yaw)


# Set up logging
logging.basicConfig(level=logging.ERROR)

# URI for Crazyflie over Crazyradio PA
URI = "radio://0/80/2M"  # Adjust channel and datarate if needed

cb_received = threading.Event()
channel = None
dataPacket = None
b = False

def appchannel_callback(data): 
    hex_values = " ".join("{:02x}".format(b) for b in data)
    print("Received appchannel data (hex):", hex_values)

    print(f"XPosition: {struct.unpack('<f', data[0:4])[0]} m")
    print(f"YPosition: {struct.unpack('<f', data[4:8])[0]} m")
    print(f"ZPosition: {struct.unpack('<f', data[8:12])[0]} m")

    print(f"Battery Voltage: {struct.unpack('<f', data[12:16])[0]} V")

    print(f"CMD_Thrust: {struct.unpack('<f', data[16:20])[0]} Thrust")

    print(f"DebugState: {struct.unpack('<b', data[20:21])[0]} 1:True, 0:False")

    cb_received.set()

def thread_function():
    global b, channel, dataPacket
    while not b:
        time.sleep(1)
        if channel is None or dataPacket is None:
            return
        data = b''.join([struct.pack('<f', val) for val in dataPacket.asTuple()])
        # print(f"Sending data: {data}")
        channel.send_packet(data)
        # print("Packet sent!")
        

thread = threading.Thread(target=thread_function)

def app():
    """Connects to the Crazyflie"""
    global b, channel, dataPacket, thread

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache='./cache')
    cf.open_link('radio://0/80/2M')
    print("Link opened!")

    cf.appchannel.packet_received.add_callback(appchannel_callback)
    cb_received.set()
    print("Callback added and set")

    channel = Appchannel(cf)
    localData = DataPacket()

    loc = Localization(cf)
    loc.send_extpose([0,0,0],[0,0,0,1])

    time.sleep(3)

    # Set the desired start values
    localData.p.extend((0.0, 0.0, 0.5))
    localData.v.extend((0.1, 0.1, 0.1))
    # dataPacket.a.extend((0.1, 0.1, 0.1)) # Removed acceleration to reduce data size
    localData.yaw = 0.0
    # dataPacket.yawRate = 0.0 # Removed yawRate to reduce data size

    dataPacket = localData

    print(f"DataPacket initialized: {dataPacket.asTuple()}")

    thread.start()

    while True:
        if cb_received.is_set():
            mode = input("Send datapacket to Crazyflie (1: z-up, 2: z-down, 3: yaw-c, 4: yaw-cc, exit: exit): ")
            match mode:
                case "exit":
                    b = not b
                    break
                case "1":
                    localData.p[2] += 0.1
                case "2":
                    localData.p[2] -= 0.1
                case "3":
                    localData.yaw += 0.1
                case "4":
                    localData.yaw -= 0.1
                case _:
                    print("Invalid input. Please enter a listed input.")
                    continue
            dataPacket = localData
            cb_received.clear()
        # print(f"DataPacket updated: {dataPacket.p[2] }")

    thread.join()
    cf.close_link()

if __name__ == "__main__":
    app()