import time
import logging
import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.appchannel import Appchannel
# Set up logging
logging.basicConfig(level=logging.ERROR)

# URI for Crazyflie over Crazyradio PA
URI = "radio://0/80/2M"  # Adjust channel and datarate if needed

def imu_callback(timestamp, data, logconf):
    print(data)

def simple_connect():
    """Connects to the Crazyflie and prints battery voltage."""
    
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # Use SyncCrazyflie for automatic connection handling
    with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
        print("Connected!")

        channel = Appchannel(scf.cf)
        
        while True:
            input("Press Enter to continue...")
            channel.send_packet([0x01])

if __name__ == "__main__":
    simple_connect()

