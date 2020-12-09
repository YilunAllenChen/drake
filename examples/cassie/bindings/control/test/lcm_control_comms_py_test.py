import numpy as np
from pydrake.all import (DiagramBuilder, DrakeLcm)
from lcm_control_comms import *
from cassie_common import *


def main():
    state_rec = StateReceiver()
    assert type(state_rec) is StateReceiver
    assert state_rec.get_utime_output_port().size() == 1
    assert state_rec.get_position_output_port().size() == kCassiePositions
    assert state_rec.get_velocity_output_port().size() == kCassieVelocities
    assert state_rec.get_contact_output_port().size() == 2

    command_pub = CommandSender()
    assert type(command_pub) is CommandSender
    assert command_pub.get_utime_input_port().size() == 1
    assert command_pub.get_torque_input_port().size() == kCassieActuators
    assert command_pub.get_motor_pos_input_port().size() == kCassieActuators
    assert command_pub.get_motor_vel_input_port().size() == kCassieActuators
    assert command_pub.get_kp_input_port().size() == kCassieActuators
    assert command_pub.get_kd_input_port().size() == kCassieActuators
    assert command_pub.get_ki_input_port().size() == kCassieActuators
    assert command_pub.get_leak_input_port().size() == kCassieActuators
    assert command_pub.get_clamp_input_port().size() == kCassieActuators

    lcm = DrakeLcm()
    builder = DiagramBuilder()

    state_rec = addLcmStateReceiver(builder, lcm)
    assert type(state_rec) is StateReceiver

    command_pub = addLcmCommandSender(builder, lcm, 0.0005)
    assert type(command_pub) is CommandSender

    state, command = addLcmControlComms(builder, lcm, 0.0005)
    assert type(state) is StateReceiver
    assert type(command) is CommandSender


if __name__ == "__main__":
    main()
