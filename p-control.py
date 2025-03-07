import time
import asyncio
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

class Fly():
    def __init__(self,fly):
        print("Init fly")
        self.exit = False
        self.flying = False
        self.kp = 3
        self.fly = fly
        self.ref = [0,0,0.2]
        self.pos = [0,0,0]
        self.error = [0,0,0]
        self.loc = Localization(self.fly)
        self.cmd = MotionCommander(fly , 0.1)
        self.t1 = threading.Thread(target=self.print_thread)
        self.t1.start()
        self.init_log()

    def init_log(self):
        log_conf = LogConfig(name="kalman", period_in_ms=10)
        log_conf.add_variable("kalman.stateX", "float")
        log_conf.add_variable("kalman.stateY", "float")
        log_conf.add_variable("kalman.stateZ", "float")
        self.fly.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.pos_callback)
        log_conf.start()

    async def take_off(self):
        self.cmd.take_off()
        self.flying = True
        await self.move_to([0,0,0.2])
        self.cmd.stop()
    
    async def move_to(self, position):
        self.ref = position
        while self.error[0]>0.01 or self.error[1]>0.01 or self.error[2]>0.01:
            await asyncio.sleep(0.1)

    async def zero(self):
        self.pos = [0,0,0]
        self.loc.send_extpos(self.pos)
        while self.error[0]>0.01 or self.error[1]>0.01 or self.error[2]>0.01:
            await asyncio.sleep(0.1)


    def print_pos(self):
        print("Ref: "+str(self.ref)+", Pos: " + str(self.pos) + ", Error: " + str(self.error))

    def print_thread(self):
        while not self.exit:
            print("Ref: "+str(self.ref)+", Pos: " + str(self.pos) + ", Error: " + str(self.error))
            time.sleep(0.1)

    def land(self):
        self.cmd.land()

    def close(self):
        self.exit = True
        self.fly.close_link()
        self.t1.join()

    def control(self):
        if self.flying:
            self.error = [self.ref[0]-self.pos[0],self.ref[1]-self.pos[1],self.ref[2]-self.pos[2]]
            self.cmd.start_linear_motion(self.error[0]*self.kp,self.error[1]*self.kp,self.error[2]*self.kp)
    
    def pos_callback(self,timestamp, data, logconf):
        self.pos = [data['kalman.stateX'],data['kalman.stateY'],data['kalman.stateZ']]
        self.control()

async def firkant(fly):
    await fly.move_to([0.2,0,0.2])
    await asyncio.sleep(0.5)
    await fly.move_to([0.2,0.2,0.2])
    await asyncio.sleep(0.5)
    await fly.move_to([0,0.2,0.2])
    await asyncio.sleep(0.5)
    await fly.move_to([0,0,0.2])
    await asyncio.sleep(0.5)

async def main():
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie()) as syncfly:
        fly = Fly(syncfly.cf)
        await fly.zero()
        print("Take off")
        await fly.take_off()
        print("hover")
        await asyncio.sleep(1)
        await firkant(fly)
        await firkant(fly)
        await firkant(fly)
        fly.land()
        time.sleep(1)
        print("final:")
        fly.print_pos()
    print("Done")

if __name__ == "__main__":
    asyncio.run(main())
