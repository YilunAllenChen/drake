import sys
import time
import numpy as np
import cProfile

from pydrake.all import (ConstantVectorSource, DiagramBuilder, DrakeVisualizer,
                         FloatingBaseType, RigidBodyPlant, RigidBodyTree,
                         Simulator, VectorSystem)
from pydrake.lcm import DrakeLcm

from cassie_common import *
from lcm_control_comms import *
from mock_low_level import *

version = 0  # 0 - RPY, 1 - Quat, 2 - LQR

if version == 0:
  from qp_balance import QPController
  rtree_ctrl = getCassieTree(CassieURDFType.kStandardCassie,
                        FloatingBaseType.kRollPitchYaw)
  x = CassieFixedPointState()
  x = np.append(x[:3], x[4:])
  q = np.copy(x[:22])

# rtree = getCassieTree(CassieURDFType.kStandardCassie,
#                       FloatingBaseType.kRollPitchYaw)
rtree = getCassieTree()
plant = RigidBodyPlant(rtree,0.0005)
# x = np.array([0, 0, 0.85247505, 0.02790626, -0.02870643, 2.57250801,
#     0.02399946, -0.0353547, -0.00346795, 0.02062358, 0.77301645, 0.78279889,
#    -1.62991405, -1.64732087, -0.00499357, -0.00423917, 1.87162173, 1.8911134,
#    -0.01146018, -0.01469796, -1.90530801, -1.90977335])
x = np.array([0.0, 0.0, 0.8524750471115112,
  -0.2804698050022125, -0.0176906269043684, -0.009361587464809418, -0.9596542119979858,
  0.02399946004152298, -0.035354699939489365, -0.003467952599748969, 0.020623575896024704,
  0.7730164527893066, 0.7827988862991333, -1.6299140453338623, -1.6473208665847778,
  -0.004993573296815157, -0.004239166621118784, 1.8716217279434204, 1.8911134004592896,
  -0.011460183188319206, -0.014697963371872902, -1.9053080081939697, -1.909773349761963])
x = np.append(x, np.zeros(22))

builder = DiagramBuilder()
cassie = builder.AddSystem(plant)
setDefaultContactParams(cassie)

controller = builder.AddSystem(
    QPController(
        rtree_ctrl,
        q_nom=q,
        control_period=0.0001,
        print_period=0.05,
        sim=False,
        cost_pub=True))

mcr = builder.AddSystem(mockCommandReceiver())
mic = builder.AddSystem(mockInputController())
mss = builder.AddSystem(mockStateSender())

sr = builder.AddSystem(StateReceiver())
cs = builder.AddSystem(CommandSender())

# Setup visualizer
lcm = DrakeLcm()
visualizer = builder.AddSystem(DrakeVisualizer(tree=rtree, 
        lcm=lcm, enable_playback=True))
builder.Connect(cassie.get_output_port(0), visualizer.get_input_port(0))

for ii in range(4):
  builder.Connect(sr.get_output_port(ii), controller.get_input_port(ii))
for ii in range(9):
  builder.Connect(controller.get_output_port(ii), cs.get_input_port(ii))

for ii in range(8):
  builder.Connect(mcr.get_output_port(ii), mic.get_input_port(ii))
builder.Connect(cassie.get_output_port(0), mic.get_state_input_port())
builder.Connect(mic.get_torque_output_port(), cassie.get_input_port(0))
builder.Connect(cassie.get_output_port(0), mss.get_state_input_port())

builder.Connect(cs.get_output_port(0), mcr.get_command_input_port())
builder.Connect(mss.get_state_output_port(), sr.get_input_port(0))

# Remove mock_input_ctrl
# builder.Connect(mcr.get_ff_output_port(), cassie.get_input_port(0))
# Remove command sending
# for ii in range(8):
#   builder.Connect(controller.get_output_port(ii+1), mic.get_input_port(ii))
# for ii in range(8):
#   builder.Connect(controller.get_output_port(ii+1), mic.get_input_port(ii))
# Remove state sending
# fake_x = np.concatenate((np.zeros(3), np.ones(1), np.zeros(41)))
# fake_state = builder.AddSystem(ConstantVectorSource(fake_x))
# builder.Connect(cassie.get_output_port(0), controller.get_input_port(0))
# builder.Connect(fake_state.get_output_port(0), mic.get_state_input_port())

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(True)

# nominal standing state
cassie.set_state_vector(simulator.get_mutable_context(),x)

simulator.StepTo(5.0)

while True:
    visualizer.ReplayCachedSimulation()
