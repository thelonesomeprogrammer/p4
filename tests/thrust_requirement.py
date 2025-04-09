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
    print(f'{logconf} Data: {data}')
    # print(f'{logconf} Data[0]: {data["m1"]}')

def simple_connect():
    """Connects to the Crazyflie"""
    
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # Use SyncCrazyflie for automatic connection handling
    with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
        print("Connected!")
        channel = Appchannel(scf.cf)
        
        scf.cf.connection_requested.add_callback

        rpm_log = LogConfig(name="rpm", period_in_ms=100)
        rpm_log.add_variable("rpm.m1", "uint16_t")
        rpm_log.add_variable("rpm.m2", "uint16_t")
        rpm_log.add_variable("rpm.m3", "uint16_t")
        rpm_log.add_variable("rpm.m4", "uint16_t")
        scf.cf.log.add_config(rpm_log)
        rpm_log.data_received_cb.add_callback(imu_callback)

        pm_log = LogConfig(name="pm", period_in_ms=100)
        pm_log.add_variable("pm.batteryLevel", "uint8_t")
        scf.cf.log.add_config(pm_log)
        pm_log.data_received_cb.add_callback(imu_callback)

        rpm_log.start()
        pm_log.start()

        channel.send_packet([0x01])
        time.sleep(10)
        rpm_log.stop()
        pm_log.stop()
        
        # while True:
        #     # input("Press Enter to continue...")
        #     t = time.time
        #     if (time.time - t == 1):
        #         channel.send_packet([0x01])

if __name__ == "__main__":
    simple_connect()

