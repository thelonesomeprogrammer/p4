import time
import logging
import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.localization import Localization
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
# Set up logging
logging.basicConfig(level=logging.ERROR)

# URI for Crazyflie over Crazyradio PA
URI = "radio://0/80/2M"  # Adjust channel and datarate if needed


def imu_callback(timestamp, data, logconf):
    """
    Callback function to handle IMU data.
    
    :param timestamp: Time at which the data was logged
    :param data: Dictionary containing IMU data
    :param logconf: Log configuration object
    """
    print(f"Timestamp: {timestamp}")
    print("-" * 50)
    print(data)
    print("-" * 50)



def simple_connect():
    """Connects to the Crazyflie and prints battery voltage."""
    
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # Use SyncCrazyflie for automatic connection handling
    with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
        print("Connected!")

        log_conf = LogConfig(name="IMU", period_in_ms=100)  # Log every 100ms
        log_conf.add_variable("kalman.stateX", "float")
        log_conf.add_variable("kalman.stateY", "float")
        log_conf.add_variable("kalman.stateZ", "float")
        #log_conf.add_variable("rpm", "float")
        #log_conf.add_variable("baro", "float")

        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(imu_callback)
        log_conf.start()

        time.sleep(1)
        loc = Localization(scf.cf)
        loc.send_extpose([0,0,0],[0,0,0,1])
        cmd = MotionCommander(scf.cf, 0.1)
        cmd.take_off()
        cmd.stop()
        time.sleep(1)
        cmd.forward(0.4)
        cmd.stop()
        cmd.left(0.2)
        cmd.stop()
        cmd.back(0.2)
        cmd.stop()
        cmd.right(0.2)
        cmd.stop()
        cmd.land()

if __name__ == "__main__":
    simple_connect()

