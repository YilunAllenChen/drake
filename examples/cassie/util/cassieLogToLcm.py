from math import *
import numpy as np
from scipy.io import loadmat
import time
import sys

import lcm
import pydrake.all
from cassie_common import *
from cassie_types import *


def sendLcmSensorMsg(lc, data, index):
    msg = lcmt_cassie_sensor()
    msg.utime = data["utime"][:, index] * 1e6
    msg.imu_orientation = data["imu_orient"][:, index]
    msg.imu_angular_velocity = data["imu_angvel"][:, index]
    msg.imu_linear_acceleration = data["imu_linacc"][:, index]
    msg.imu_magnetic_field = data["imu_mag"][:, index]
    msg.motor_positions = data["motor_pos"][:, index]
    msg.motor_velocities = data["motor_vel"][:, index]
    msg.measured_torques = data["motor_torque"][:, index]
    msg.joint_positions = data["joint_pos"][:, index]
    msg.joint_velocities = data["joint_vel"][:, index]
    lc.publish("CASSIE_SENSOR", msg.encode())


def sendLcmStatusMsg(lc, data, index):
    msg = lcmt_cassie_status()
    msg.utime = data["utime"][:, index] * 1e6
    msg.charge_state = data["soc"][:, index] * 100
    msg.radio = data["radio"][:, index]
    msg.pressure = data["press"][:, index]
    msg.temperature = data["temp"][:, index]
    lc.publish("CASSIE_STATUS", msg.encode())


def sendLcmCommandMsg(lc, data, index):
    msg = lcmt_cassie_command()
    msg.utime = data["utime"][:, index] * 1e6
    msg.motor_torques = data["torque"][:, index]
    lc.publish("CASSIE_COMMAND", msg.encode())


def main():
    if len(sys.argv) < 2:
        raise Exception("No file given")
    log_file = sys.argv[1]

    file_type = log_file[-3:]
    if file_type != "mat" and file_type != "csv":
        raise Exception("Only supports .mat or .csv log files")

    print "Reading log data from file"
    if file_type == "csv":
        data = np.genfromtxt(log_file, delimiter=',')
        log_data = dict()
        log_data["utime"] = data[0].reshape((1, -1))
        log_data["imu_orient"] = data[1:5]
        log_data["imu_angvel"] = data[5:8]
        log_data["imu_linacc"] = data[8:11]
        log_data["imu_mag"] = data[11:14]
        log_data["motor_pos"] = data[14:24]
        log_data["motor_vel"] = data[24:34]
        log_data["motor_torque"] = data[34:44]
        log_data["joint_pos"] = data[[44, 45, 47, 48]]
        log_data["joint_vel"] = data[[50, 51, 53, 54]]
        log_data["radio"] = data[56:72]
        log_data["soc"] = data[72].reshape((1, -1))
        log_data["press"] = data[73].reshape((1, -1))
        log_data["temp"] = data[74].reshape((1, -1))
        log_data["torque"] = data[75:]
    elif file_type == "mat":
        log_data = loadmat(log_file)
        log_data["joint_pos"] = log_data["joint_pos"][[0, 1, 3, 4]]
        log_data["joint_vel"] = log_data["joint_vel"][[0, 1, 3, 4]]

    print "Starting Lcm publishing"
    lc = lcm.LCM()
    t_start = time.time()
    for ii in range(log_data["utime"].shape[1]):
        sendLcmSensorMsg(lc, log_data, ii)
        sendLcmCommandMsg(lc, log_data, ii)
        if ii % 10 == 0:
            sendLcmStatusMsg(lc, log_data, ii)
        while time.time() < t_start + 0.0005:
            pass
        t_start = time.time()


if __name__ == '__main__':
    main()
