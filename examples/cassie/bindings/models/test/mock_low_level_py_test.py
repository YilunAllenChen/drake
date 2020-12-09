import numpy as np
from pydrake.all import (DiagramBuilder, DrakeLcm)
from mock_low_level import *
from cassie_common import *


def main():
    command_rec = mockCommandReceiver()
    assert type(command_rec) is mockCommandReceiver
    command_rec.get_command_input_port()
    assert command_rec.get_ff_output_port().size() == kCassieActuators
    assert command_rec.get_q_d_output_port().size() == kCassieActuators
    assert command_rec.get_qdot_d_output_port().size() == kCassieActuators
    assert command_rec.get_kp_output_port().size() == kCassieActuators
    assert command_rec.get_kd_output_port().size() == kCassieActuators
    assert command_rec.get_ki_output_port().size() == kCassieActuators
    assert command_rec.get_leak_output_port().size() == kCassieActuators
    assert command_rec.get_clamp_output_port().size() == kCassieActuators

    input_ctrl = mockInputController()
    assert type(input_ctrl) is mockInputController
    assert input_ctrl.get_ff_input_port().size() == kCassieActuators
    assert input_ctrl.get_q_d_input_port().size() == kCassieActuators
    assert input_ctrl.get_qdot_d_input_port().size() == kCassieActuators
    assert input_ctrl.get_kp_input_port().size() == kCassieActuators
    assert input_ctrl.get_kd_input_port().size() == kCassieActuators
    assert input_ctrl.get_ki_input_port().size() == kCassieActuators
    assert input_ctrl.get_leak_input_port().size() == kCassieActuators
    assert input_ctrl.get_clamp_input_port().size() == kCassieActuators
    assert input_ctrl.get_state_input_port().size() == kCassieStates
    assert input_ctrl.get_torque_output_port().size() == kCassieActuators

    sensor_send = mockSensorSender()
    assert type(sensor_send) is mockSensorSender
    assert sensor_send.get_sensor_input_port().size() == kCassieStates
    assert sensor_send.get_torque_input_port().size() == kCassieActuators
    sensor_send.get_sensor_output_port()
    sensor_send.get_status_output_port()

    state_send = mockStateSender()
    assert type(state_send) is mockStateSender
    assert state_send.get_state_input_port().size() == kCassieStates
    state_send.get_state_output_port()


if __name__ == "__main__":
    main()
