from math import *
import time
import numpy as np
import lcm
from cassie_types import *

lc = lcm.LCM()


class StatueStand:
    def __init__(self):
        self.chanIn = "CASSIE_SENSOR"
        # self.chanIn = "CASSIE_STATE"
        self.chanOut = "CASSIE_COMMAND"
        self.tStart = -1

    def state_handle(self, channel, data):
        msgState = lcmt_cassie_sensor.decode(data)
        # msgState = lcmt_cassie_state.decode(data)
        # kp = [400, 60, 200, 390, 200, 520, 100, 200, 430, 250]
        kp = [300, 50, 200, 350, 50, 300, 50, 200, 350, 50]
        # kp = np.divide(kp, 50)
        # kd = [0.25,0.25,0.75,0.75,0.75,0.5,0.25,0.75,0.75,0.75]
        kd = 0.5 * np.sqrt(kp)
        q_d = [0, 0.0057, 0.6726, -1.4100, -1.7479,
               0, 0.0057, 0.6726, -1.4100, -1.7479]

        msgIn = lcmt_cassie_command()
        msgIn.utime = msgState.utime
        if self.tStart < 0:
            self.tStart = msgState.utime

        # msgIn.motor_torques = [2,0,6,-6,0,-2,0,6,-6,0]
        msgIn.motor_kp = kp
        msgIn.motor_kd = kd
        msgIn.integrator_clamp = [50]*10

        if msgState.utime - self.tStart > 10*1000000:
            msgIn.leak_factor = [1]*10
            msgIn.motor_ki = [100]*10

        msgIn.motor_positions_desired = q_d

        lc.publish(self.chanOut, msgIn.encode())


def main():
    obj = StatueStand()
    lc.subscribe(obj.chanIn, obj.state_handle)
    while True:
        lc.handle()


if __name__ == '__main__':
    main()


# Toes
    # L - kp:30   kd:0.75
    # R - kp:37.5 kd:0.75
# Knees
    # L - kp: 45  kd:0.75 ff:-6
    # R - kp: 50  kd:0.75 ff:-6
# Hips
    # L - kp: 40  kd:0.75 ff:6
    # R - kp: 40  kd:0.75 ff:6
# Yaw
    # L - kp:60   kd:0.25
    # R - kp:100  kd:0.25
# Abd
    # L - kp: 100 kd:0.25 ff:2
    # R - kp: 130 kd:0.5  ff:-2
