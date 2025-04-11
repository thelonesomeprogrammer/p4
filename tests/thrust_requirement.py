import time
import logging
import struct
import threading
from dataclasses import dataclass
import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.appchannel import Appchannel

class DataPacket:
    battery_voltage: float
    pressure_max: float
    pressure_min: float
    pressure: float
    procent: int

# Set up logging
logging.basicConfig(level=logging.ERROR)

# URI for Crazyflie over Crazyradio PA
URI = "radio://0/80/2M"  # Adjust channel and datarate if needed

dataPacket = DataPacket()
cb_received = threading.Event()

def appchannel_callback(data): 
    hex_values = " ".join("{:02x}".format(b) for b in data)
    print("Received appchannel data (hex):", hex_values)
    dataPacket.battery_voltage = struct.unpack('<f', data[0:4])[0]
    print(f"Battery Voltage: {dataPacket.battery_voltage} V")

    dataPacket.pressure_max = struct.unpack('<f', data[4:8])[0]
    print(f"Pressure Max: {dataPacket.pressure_max} mbar")
    dataPacket.pressure_min = struct.unpack('<f', data[8:12])[0]
    print(f"Pressure Min: {dataPacket.pressure_min} mbar")
    dataPacket.pressure = struct.unpack('<f', data[12:16])[0]
    print(f"Pressure: {dataPacket.pressure} mbar \n")

    dataPacket.procent = struct.unpack('<i', data[16:20])[0]
    print(f"Procent: {dataPacket.procent} %")

    cb_received.set()

def simple_connect():
    """Connects to the Crazyflie"""
    
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache='./cache')
    cf.open_link('radio://0/80/2M')
    print("Link opened!")

    cf.appchannel.packet_received.add_callback(appchannel_callback)
    print("Callback added")

    channel = Appchannel(cf)
    print("Sending sample request!")
    channel.send_packet([0x00, 0x01])

    while True:
        if cb_received.is_set():
            signal = input("Press Enter to send packet...")
            cb_received.clear()

            if (signal == "0"):
                channel.send_packet([0x00, 0x01])
            elif (signal == "1"):
                channel.send_packet([0x01, 0x00])

    # cf.close_link()

if __name__ == "__main__":
    simple_connect()