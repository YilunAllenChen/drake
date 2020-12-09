import os.path
from pydrake.all import (DiagramBuilder, FloatingBaseType, RigidBodyPlant,
                         RigidBodyTree, Simulator, VectorSystem,
                         ConstantVectorSource, CompliantMaterial,
                         CompliantContactModelParameters, DrakeVisualizer,
                         AddFlatTerrainToWorld)

from pydrake.multibody.rigid_body_tree import (FloatingBaseType,
                                               RigidBodyFrame, RigidBodyTree)

from pydrake.lcm import DrakeLcm
from cassie_common import *
import numpy as np


rtree = getCassieTree()
plant = RigidBodyPlant(rtree, 0.0005)

builder = DiagramBuilder()
cassie = builder.AddSystem(plant)
setDefaultContactParams(cassie)

# precomputed standing fixed point state
# q = getNominalStandingConfiguration()
# qd = [0. for i in xrange(rtree.get_num_velocities())]
x = CassieFixedPointState()

# Setup visualizer
lcm = DrakeLcm()
visualizer = builder.AddSystem(DrakeVisualizer(tree=rtree, lcm=lcm,
                                               enable_playback=True))

builder.Connect(cassie.get_output_port(0), visualizer.get_input_port(0))

# Zero inputs -- passive forward simulation
u0 = ConstantVectorSource(np.zeros(rtree.get_num_actuators()))
null_controller = builder.AddSystem(u0)

builder.Connect(null_controller.get_output_port(0), cassie.get_input_port(0))

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(True)

# kinsol = rtree.doKinematics(q)
# com = rtree.centerOfMass(kinsol)
# print com

# nominal standing state
# state = simulator.get_mutable_context().get_mutable_continuous_state_vector()
# state.SetFromVector(q+qd)
cassie.set_state_vector(simulator.get_mutable_context(), x)

simulator.StepTo(1.0)

# simulator.Initialize();
# from bot_lcmgl import lcmgl
# import lcm as true_lcm
# viz_lcmgl = lcmgl("Visualize-Points", true_lcm.LCM())
# cache = rtree.doKinematics(x[:23], x[23:])

# thigh_l = rtree.transformPoints(cache, GetFourBarHipMountPoint(),
#                                 rtree.FindBodyIndex("thigh_left"), 0)
# heel_l = rtree.transformPoints(cache, GetFourBarHeelMountPoint(),
#                                rtree.FindBodyIndex("heel_spring_left"), 0)
# thigh_r = rtree.transformPoints(cache, -GetFourBarHipMountPoint(),
#                                 rtree.FindBodyIndex("thigh_right"), 0)
# heel_r = rtree.transformPoints(cache, GetFourBarHeelMountPoint(),
#                                rtree.FindBodyIndex("heel_spring_right"), 0)

# viz_lcmgl.glColor3f(0, 0, 1)
# viz_lcmgl.sphere(thigh_l[0], thigh_l[1], thigh_l[2], 0.005, 20, 20)
# viz_lcmgl.sphere(heel_l[0], heel_l[1], heel_l[2], 0.005, 20, 20)
# viz_lcmgl.sphere(thigh_r[0], thigh_r[1], thigh_r[2], 0.005, 20, 20)
# viz_lcmgl.sphere(heel_r[0], heel_r[1], heel_r[2], 0.005, 20, 20)
# viz_lcmgl.switch_buffer()

# toe_l = rtree.transformPoints(cache, [0.0211, 0.0560, 0],
#                               rtree.FindBodyIndex("toe_left"), 0)
# toe_r = rtree.transformPoints(cache, [0.0211, 0.0560, 0],
#                               rtree.FindBodyIndex("toe_right"), 0)
# toe_l_ar = rtree.transformPoints(cache, [0.01762, 0.05219, 0],
#                                  rtree.FindBodyIndex("toe_left"), 0)
# toe_r_ar = rtree.transformPoints(cache, [0.01762, 0.05219, 0],
#                                  rtree.FindBodyIndex("toe_right"), 0)
# print toe_l
# print toe_l_ar

# viz_lcmgl.glColor3f(0, 1, 0)
# viz_lcmgl.sphere(toe_l[0], toe_l[1], toe_l[2], 0.005, 20, 20)
# viz_lcmgl.sphere(toe_r[0], toe_r[1], toe_r[2], 0.005, 20, 20)
# viz_lcmgl.glColor3f(0, 0, 1)
# viz_lcmgl.sphere(toe_l_ar[0], toe_l_ar[1], toe_l_ar[2], 0.005, 20, 20)
# viz_lcmgl.sphere(toe_r_ar[0], toe_r_ar[1], toe_r_ar[2], 0.005, 20, 20)
# viz_lcmgl.switch_buffer()
