import sys
import time
import numpy as np
import cProfile

from pydrake.all import (ConstantVectorSource, DiagramBuilder, DrakeVisualizer,
                         FloatingBaseType, RigidBodyPlant, RigidBodyTree,
                         Simulator, VectorSystem)
from pydrake.lcm import DrakeLcm

from cassie_common import *

def profileCalcVec(controller,x):
    context = controller.CreateDefaultContext()
    discrete_state = context.get_state().get_discrete_state()
    context.FixInputPort(0,x)
    for ii in range(1000):
        controller._DoCalcDiscreteVariableUpdates(context, None, discrete_state)

version = 0  # 0 - RPY, 1 - Quat, 2 - LQR

if version == 0:
  from qp_balance import QPController
  rtree = getCassieTree(CassieURDFType.kStandardCassie,
                        FloatingBaseType.kRollPitchYaw)
  x = CassieFixedPointState()
  x = np.append(x[:3], x[4:])
  q = np.copy(x[:22])
  x[22] = 0.2

  x = np.array([0, 0, 0.85247505, 0.02790626, -0.02870643, 2.57250801,
    0.02399946, -0.0353547, -0.00346795, 0.02062358, 0.77301645, 0.78279889,
   -1.62991405, -1.64732087, -0.00499357, -0.00423917, 1.87162173, 1.8911134,
   -0.01146018, -0.01469796, -1.90530801, -1.90977335])
  x = np.append(x, np.zeros(22))

plant = RigidBodyPlant(rtree,0.0005)

builder = DiagramBuilder()
cassie = builder.AddSystem(plant)
setDefaultContactParams(cassie)

controller = builder.AddSystem(
    QPController(
        rtree,
        q_nom=q,
        control_period=0.0001,
        print_period=0.05,
        sim=True,
        cost_pub=True))

# start = time.time()
# context = controller.CreateDefaultContext()
# discrete_state = context.get_state().get_discrete_state()
# context.FixInputPort(0,x)
# controller._DoCalcDiscreteVariableUpdates(context, None, discrete_state)
# # profileCalcVec(controller,x)
# cProfile.run('profileCalcVec(controller,x)',sort='tottime')
# end = time.time()
# print end-start

# Setup visualizer
lcm = DrakeLcm()
visualizer = builder.AddSystem(DrakeVisualizer(tree=rtree, 
        lcm=lcm, enable_playback=True))
builder.Connect(cassie.get_output_port(0), visualizer.get_input_port(0))

builder.Connect(controller.get_output_port(0), cassie.get_input_port(0))
builder.Connect(cassie.get_output_port(0), controller.get_input_port(0))

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(True)

# nominal standing state
cassie.set_state_vector(simulator.get_mutable_context(),x)

simulator.StepTo(5.0)

while True:
    visualizer.ReplayCachedSimulation()
