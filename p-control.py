import time
import cflib
import threading
import logging
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.localization import Localization
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig

logging.basicConfig(level=logging.ERROR)

# URI for Crazyflie over Crazyradio PA
URI = "radio://0/80/2M"

class fly():
    def __init__(self,fly):
        print("Init fly")
        self.pri = 0
        self.fly = fly
        self.ref = [0,0,0.2]
        self.init_log()
        self.loc = Localization(self.fly)
        self.pos = [0,0,0]
        self.error = [0,0,0]
        self.loc.send_extpos(self.pos)
        self.cmd = MotionCommander(fly , 0.1)

    def init_log(self):
        log_conf = LogConfig(name="kalman", period_in_ms=10)
        log_conf.add_variable("kalman.stateX", "float")
        log_conf.add_variable("kalman.stateY", "float")
        log_conf.add_variable("kalman.stateZ", "float")
        self.fly.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.pos_callback)
        log_conf.start()

    def take_off(self):
        self.cmd.take_off()
        self.cmd.stop()
    
    def move_to(self, position):
        self.ref = position

    def print_pos(self):
        print("Ref: "+str(self.ref)+", Pos: " + str(self.pos) + ", Error: " + str(self.error))


    def land(self):
        self.cmd.land()

    def send_extpose(self, position, orientation):
        self.loc.send_extpose(position, orientation)

    def close(self):
        self.fly.close_link()

    def control(self):
        self.pri=self.pri+1
        kp = 2
        self.error = [self.ref[0]-self.pos[0],self.ref[1]-self.pos[1],self.ref[2]-self.pos[2]]
        if self.pri%100==0:
            print("Ref: "+str(self.ref)+", Pos: " + str(self.pos) + ", Error: " + str(self.error))
        self.cmd.start_linear_motion(self.error[0]*kp,self.error[1]*kp,self.error[2]*kp)
    
    def pos_callback(self,timestamp, data, logconf):
        self.pos = [data['kalman.stateX'],data['kalman.stateY'],data['kalman.stateZ']]
        try:
            self.control()
        except:
            pass


if __name__ == "__main__":
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie()) as syncfly:
        fly = fly(syncfly.cf)
        print("Take off")
        fly.take_off()
        print("hover")
        for i in range(500):
            time.sleep(0.01)
        fly.land()
        time.sleep(1)
        print("final:")
        fly.print_pos()
    print("Done")

