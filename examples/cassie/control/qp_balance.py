import argparse
import math
import functools
import os.path
import time
import random
import sys

from math import *
import numpy as np
import numpy.matlib
import scipy as sp
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from pydrake.all import (BasicVector, GurobiSolver, LeafSystem,
                         MathematicalProgram, OsqpSolver, PortDataType,
                         Quaternion, RigidBodyTree, RollPitchYaw,
                         SolutionResult, SolverType)

from cassie_common import *
from control_util import *
from cassie_types import *
import lcm as true_lcm
from bot_lcmgl import lcmgl
from math import *

# TODO(Mark P): Create bindings for cassie_lcm_parsing
kCassieMotorUrdfToSimulink = np.array([0,2,4,6,8,1,3,5,7,9])
lc = true_lcm.LCM()

class QPController(LeafSystem):
    ''' This system consumes state from a simulation
        of Cassie and produces torque
        commands for each motor. '''
    
    ''' 
    The class constructor sets up the input and output
    information of the system.
    Arguments:
    - rtree: The Cassie RigidBodyTree
    - q_nom: The nominal configuration for the robot
    - control_period: The update rate of the controller. This
        system will produce output as quickly as desired,
        but will only compute new control actions at this rate.
    - print_period: The controller will print to the terminal
        at this period (in simulation seconds) as a progress update.
    '''
    def __init__(self, rtree, q_nom, control_period = 0.001,
                 print_period=1.0, sim=True, cost_pub=False):
                
        LeafSystem.__init__(self)

        self.rtree = rtree
        self.nq = rtree.get_num_positions()
        self.nv = rtree.get_num_velocities()
        self.nu = rtree.get_num_actuators()
        self.cost_pub = cost_pub

        dim = 3 # 3D
        nd = 4 # for friction cone approx, hard coded for now
        self.nc = 4 # number of contacts; TODO don't hard code

        self.nf = self.nc*nd # number of contact force variables
        self.ncf = 2 # number of loop constraint forces; TODO don't hard code
        self.neps = self.nc*dim # number of slack variables for contact

        if sim:
            self.sim = True
            self._DeclareInputPort(PortDataType.kVectorValued, self.nq+self.nv)
            self._DeclareDiscreteState(self.nu)
            self._DeclareVectorOutputPort(BasicVector(self.nu), self.CopyStateOutSim)

            self.print_period = print_period
            self.last_print_time = -print_period

            self.last_v = np.zeros(3)
            self.lcmgl = lcmgl("Balancing-plan", true_lcm.LCM())
            # self.lcmgl = None
        else:
            self.sim = False
            self._DeclareInputPort(PortDataType.kVectorValued, 1)
            self._DeclareInputPort(PortDataType.kVectorValued, kCassiePositions)
            self._DeclareInputPort(PortDataType.kVectorValued, kCassieVelocities)
            self._DeclareInputPort(PortDataType.kVectorValued, 2)
            self._DeclareDiscreteState(kCassieActuators*8 + 1)
            self._DeclareVectorOutputPort(BasicVector(1),
                    lambda c,o: self.CopyStateOut(c, o, 8*kCassieActuators, 1))
            self._DeclareVectorOutputPort(BasicVector(kCassieActuators),
                    lambda c,o: self.CopyStateOut(c, o, 0, kCassieActuators)) #torque
            self._DeclareVectorOutputPort(BasicVector(kCassieActuators),
                    lambda c,o: self.CopyStateOut(c, o, kCassieActuators, kCassieActuators)) #motor_pos
            self._DeclareVectorOutputPort(BasicVector(kCassieActuators),
                    lambda c,o: self.CopyStateOut(c, o, 2*kCassieActuators, kCassieActuators)) #motor_vel
            self._DeclareVectorOutputPort(BasicVector(kCassieActuators),
                    lambda c,o: self.CopyStateOut(c, o, 3*kCassieActuators, kCassieActuators)) #kp
            self._DeclareVectorOutputPort(BasicVector(kCassieActuators),
                    lambda c,o: self.CopyStateOut(c, o, 4*kCassieActuators, kCassieActuators)) #kd
            self._DeclareVectorOutputPort(BasicVector(kCassieActuators),
                    lambda c,o: self.CopyStateOut(c, o, 5*kCassieActuators, kCassieActuators)) #ki
            self._DeclareVectorOutputPort(BasicVector(kCassieActuators),
                    lambda c,o: self.CopyStateOut(c, o, 6*kCassieActuators, kCassieActuators)) #leak
            self._DeclareVectorOutputPort(BasicVector(kCassieActuators),
                    lambda c,o: self.CopyStateOut(c, o, 7*kCassieActuators, kCassieActuators)) #clamp
        self._DeclarePeriodicDiscreteUpdate(0.0005);

        self.q_nom = q_nom
        self.com_des = rtree.centerOfMass(self.rtree.doKinematics(q_nom))
        self.u_des = CassieFixedPointTorque()

        self.lfoot = rtree.FindBody("toe_left")
        self.rfoot = rtree.FindBody("toe_right")
        self.springs = 2 + np.array(
            [rtree.FindIndexOfChildBodyOfJoint("knee_spring_left_fixed"),
             rtree.FindIndexOfChildBodyOfJoint("knee_spring_right_fixed"),
             rtree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_left"),
             rtree.FindIndexOfChildBodyOfJoint("ankle_spring_joint_right")])

        umin = np.zeros(self.nu)
        umax = np.zeros(self.nu)
        ii = 0
        for motor in rtree.actuators:
            umin[ii] = motor.effort_limit_min
            umax[ii] = motor.effort_limit_max
            ii += 1

        slack_limit = 10.0
        self.initialized = False

        #------------------------------------------------------------
        # Add Decision Variables ------------------------------------
        prog = MathematicalProgram()
        qddot = prog.NewContinuousVariables(self.nq, "joint acceleration")
        u = prog.NewContinuousVariables(self.nu, "input")
        bar = prog.NewContinuousVariables(self.ncf, "four bar forces")
        beta = prog.NewContinuousVariables(self.nf, "friction forces")
        eps = prog.NewContinuousVariables(self.neps, "slack variable")
        nparams = prog.num_vars()

        #------------------------------------------------------------
        # Problem Constraints ---------------------------------------
        self.con_u_lim = prog.AddBoundingBoxConstraint(
            umin, umax, u).evaluator()
        self.con_fric_lim = prog.AddBoundingBoxConstraint(
            0, 100000, beta).evaluator()
        self.con_slack_lim = prog.AddBoundingBoxConstraint(
            -slack_limit, slack_limit, eps).evaluator()

        bar_con = np.zeros((self.ncf, self.nq))
        self.con_4bar = prog.AddLinearEqualityConstraint(bar_con,
            np.zeros(self.ncf), qddot).evaluator()

        if self.nc > 0:
            dyn_con = np.zeros((self.nq,self.nq+self.nu+self.ncf+self.nf))
            dyn_vars = np.concatenate((qddot,u,bar,beta))
            self.con_dyn = prog.AddLinearEqualityConstraint(dyn_con,
                np.zeros(self.nq), dyn_vars).evaluator()

            foot_con = np.zeros((self.neps, self.nq+self.neps))
            foot_vars = np.concatenate((qddot,eps))
            self.con_foot = prog.AddLinearEqualityConstraint(foot_con,
                np.zeros(self.neps), foot_vars).evaluator()

        else:
            dyn_con = np.zeros((self.nq,self.nq+self.nu+self.ncf))
            dyn_vars = np.concatenate((qddot,u,bar))
            self.con_dyn = prog.AddLinearEqualityConstraint(dyn_con,
                np.zeros(self.nq), dyn_vars).evaluator()

        #------------------------------------------------------------
        # Problem Costs ---------------------------------------------
        self.Kp_com = 50
        self.Kd_com = 2.0*sqrt(self.Kp_com)
        # self.Kp_qdd = 10*np.eye(self.nq)#np.diag(np.diag(H)/np.max(np.diag(H)))
        self.Kp_qdd = np.diag(np.concatenate(([10,10,10,5,5,5], np.zeros(self.nq-6))))
        self.Kd_qdd = 1.0*np.sqrt(self.Kp_qdd)
        self.Kp_qdd[self.springs,self.springs] = 0
        self.Kd_qdd[self.springs,self.springs] = 0

        com_ddot_des = np.zeros(dim)
        qddot_des = np.zeros(self.nq)

        self.w_com = 0
        self.w_qdd = np.zeros(self.nq)
        self.w_qdd[:6] = 10
        self.w_qdd[8:10] = 1
        self.w_qdd[3:6] = 1000
        # self.w_qdd[self.springs] = 0
        self.w_u = 0.001*np.ones(self.nu)
        self.w_u[2:4] = 0.01
        self.w_slack = 0.1

        self.cost_qdd = prog.AddQuadraticErrorCost(
            np.diag(self.w_qdd), qddot_des, qddot).evaluator()
        self.cost_u = prog.AddQuadraticErrorCost(
            np.diag(self.w_u), self.u_des, u).evaluator()
        self.cost_slack = prog.AddQuadraticErrorCost(
            self.w_slack*np.eye(self.neps), np.zeros(self.neps), eps).evaluator()

        # self.cost_com = prog.AddQuadraticCost().evaluator()
        # self.cost_qdd = prog.AddQuadraticCost(
        #     2*np.diag(self.w_qdd), -2*np.matmul(qddot_des, np.diag(self.w_qdd)), qddot).evaluator()
        # self.cost_u = prog.AddQuadraticCost(
        #     2*np.diag(self.w_u), -2*np.matmul(self.u_des, np.diag(self.w_u)) , u).evaluator()
        # self.cost_slack = prog.AddQuadraticCost(
        #     2*self.w_slack*np.eye(self.neps), np.zeros(self.neps), eps).evaluator()

        REG = 1e-8
        self.cost_reg = prog.AddQuadraticErrorCost(REG*np.eye(nparams),
            np.zeros(nparams), prog.decision_variables()).evaluator()
        # self.cost_reg = prog.AddQuadraticCost(2*REG*np.eye(nparams),
        #     np.zeros(nparams), prog.decision_variables()).evaluator()

        #------------------------------------------------------------
        # Solver settings -------------------------------------------
        self.solver = GurobiSolver()
        # self.solver = OsqpSolver()
        prog.SetSolverOption(SolverType.kGurobi, "Method", 2)

        #------------------------------------------------------------
        # Save MathProg Variables -----------------------------------
        self.qddot = qddot
        self.u = u
        self.bar = bar
        self.beta = beta
        self.eps = eps
        self.prog = prog

    def CopyStateOut(self, context, output, start, data_len):
        x = context.get_discrete_state_vector().get_value()
        y = output.get_mutable_value()
        y[:] = x[start:start+data_len]

    def CopyStateOutSim(self, context, output):
        # self._DoCalcDiscreteVariableUpdates(context, None, None)
        x = context.get_discrete_state_vector().get_value()
        y = output.get_mutable_value()
        y[:] = x

    def packLcmCommand(self, utime, torque, motor_pos, motor_vel, kp, kd, ki,
                       leak, clamp):
        msgCommand = lcmt_cassie_command()
        msgCommand.utime = utime
        for ii in range(kCassieActuators):
            sim_ii = kCassieMotorUrdfToSimulink[ii]
            msgCommand.motor_torques[ii] = torque[sim_ii]
            msgCommand.motor_positions_desired[ii] = motor_pos[sim_ii]
            msgCommand.motor_velocities_desired[ii] = motor_vel[sim_ii]
            msgCommand.motor_kp[ii] = kp[sim_ii]
            msgCommand.motor_kd[ii] = kd[sim_ii]
            msgCommand.motor_ki[ii] = ki[sim_ii]
            msgCommand.leak_factor[ii] = leak[sim_ii]
            msgCommand.integrator_clamp[ii] = clamp[sim_ii]
        return msgCommand

    def packLcmCost(self, utime, qddot, qddot_des, u, eps):
        msgCost = lcmt_data()
        msgCost.utime = utime
        msgCost.dim = 9
        data = np.zeros(msgCost.dim)
        for ii in range(6):
            data[ii] = self.w_qdd[ii] * (qddot[ii] - qddot_des[ii])**2
        data[6] = (self.u_des-u).dot(np.diag(self.w_u)).dot((self.u_des-u))
        data[7] = self.w_slack * eps.dot(eps)
        data[8] = np.sum(data[:8])
        msgCost.data = data
        return msgCost

    def _DoCalcDiscreteVariableUpdates(self, context, events, discrete_state):
        y = discrete_state.get_mutable_vector().get_mutable_value()

        if self.sim:
            x = self.EvalVectorInput(context, 0).CopyToVector()
            q = x[0:self.nq]
            qd = x[self.nq:]
            utime = context.get_time()*1000000
        else:
            utime = self.EvalVectorInput(context, 0).CopyToVector()
            q = self.EvalVectorInput(context, 1).CopyToVector()
            v = self.EvalVectorInput(context, 2).CopyToVector()
            # foot_contact = self.EvalVectorInput(context, 3).CopyToVector()

            # Convert from quaternions to Euler Angles
            rpy = RollPitchYaw(Quaternion(q[3:7]))
            R_rpy = rpy.ToRotationMatrix().matrix()
            q = np.concatenate((q[:3], rpy.vector(), q[7:]))
            qd = np.concatenate((np.matmul(R_rpy, v[3:6]), v[:3], v[6:]))

        r = self.rtree
        kinsol = r.doKinematics(q,qd)

        if not self.initialized:
            toe_l = r.transformPoints(kinsol, p_midfoot, r.FindBodyIndex("toe_left"), 0)
            toe_r = r.transformPoints(kinsol, p_midfoot, r.FindBodyIndex("toe_right"), 0)
            offset = np.array([1.28981386e-02, 2.02279085e-08])
            yaw = q[5]
            yaw_rot = np.array([[cos(yaw), -sin(yaw)], [sin(yaw), cos(yaw)]])
            self.q_nom[:2] = (toe_l[:2,0] + toe_r[:2,0])/2 - np.matmul(yaw_rot, offset)
            self.q_nom[5] = yaw
            print self.q_nom[:6]
            self.initialized = True
            # self.lcmgl.glColor3f(1, 0, 0)
            # self.lcmgl.sphere(q[0], q[1], 0, 0.01, 20, 20)
            # self.lcmgl.glColor3f(0, 0, 1)
            # self.lcmgl.sphere(self.q_nom[0], self.q_nom[1], 0, 0.01, 20, 20)
            # self.lcmgl.glColor3f(0, 1, 0)
            # self.lcmgl.sphere((toe_l[0,0] + toe_r[0,0])/2, (toe_l[1,0] + toe_r[1,0])/2, 0, 0.01, 20, 20)
            # self.lcmgl.switch_buffer()
            # raise Exception()

        H = r.massMatrix(kinsol)
        C = r.dynamicsBiasTerm(kinsol, {}, None)
        B = r.B

        com = r.centerOfMass(kinsol)
        Jcom = r.centerOfMassJacobian(kinsol) 
        Jcomdot_times_v = r.centerOfMassJacobianDotTimesV(kinsol)

        phi = contactDistancesAndNormals(self.rtree, kinsol, self.lfoot, self.rfoot)
        [force_vec,JB] = contactJacobianBV(self.rtree, kinsol, self.lfoot, self.rfoot, False)
        # TODO: Look into contactConstraintsBV

        Jp = contactJacobian(self.rtree, kinsol, self.lfoot, self.rfoot, False)
        Jpdot_times_v = contactJacobianDotTimesV(self.rtree, kinsol, self.lfoot, self.rfoot)

        J4bar = r.positionConstraintsJacobian(kinsol, False)
        J4bardot_times_v = r.positionConstraintsJacDotTimesV(kinsol)

        #------------------------------------------------------------
        # Problem Constraints ---------------------------------------
        self.con_4bar.UpdateCoefficients(J4bar, -J4bardot_times_v)

        if self.nc > 0:
            dyn_con = np.zeros((self.nq,self.nq+self.nu+self.ncf+self.nf))
            dyn_con[:, :self.nq] = H
            dyn_con[:, self.nq:self.nq+self.nu] = -B
            dyn_con[:, self.nq+self.nu:self.nq+self.nu+self.ncf] = -J4bar.T
            dyn_con[:, self.nq+self.nu+self.ncf:] = -JB
            self.con_dyn.UpdateCoefficients(dyn_con, -C)

            foot_con = np.zeros((self.neps, self.nq+self.neps))
            foot_con[:, :self.nq] = Jp
            foot_con[:, self.nq:] = -np.eye(self.neps)
            self.con_foot.UpdateCoefficients(foot_con, -Jpdot_times_v)

        else:
            dyn_con = np.zeros((self.nq,self.nq+self.nu+self.ncf))
            dyn_con[:, :self.nq] = H
            dyn_con[:, self.nq:self.nq+self.nu] = -B
            dyn_con[:, self.nq+self.nu:] = -J4bar.T
            self.con_dyn.UpdateCoefficients(dyn_con, -C)

        #------------------------------------------------------------
        # Problem Costs ---------------------------------------------
        com_ddot_des = self.Kp_com*(self.com_des - com) - self.Kd_com*np.matmul(Jcom, qd)
        qddot_des = np.matmul(self.Kp_qdd, self.q_nom-q) - np.matmul(self.Kd_qdd, qd)

        self.cost_qdd.UpdateCoefficients(2*np.diag(self.w_qdd),
            -2*np.matmul(qddot_des, np.diag(self.w_qdd)))
        self.cost_u.UpdateCoefficients(2*np.diag(self.w_u),
            -2*np.matmul(self.u_des, np.diag(self.w_u)))
        self.cost_slack.UpdateCoefficients(2*self.w_slack*np.eye(self.neps),
            np.zeros(self.neps))


        result = self.solver.Solve(self.prog)
        # print result
        # print self.prog.GetSolverId().name()
        y[:self.nu] = self.prog.GetSolution(self.u)

        # np.set_printoptions(precision=4, suppress=True, linewidth=1000)
        # print qddot_des
        # print self.prog.GetSolution(self.qddot)
        # print self.prog.GetSolution(self.u)
        # print self.prog.GetSolution(self.beta)
        # print self.prog.GetSolution(self.bar)
        # print self.prog.GetSolution(self.eps)
        # print
        # return

        if not self.sim:
            y[8*self.nu] = utime
            y[self.nu:8*self.nu] = np.zeros(7*self.nu)
            return

        if (self.sim and self.lcmgl is not None):
            grf = np.zeros((3,self.nc))
            for ii in range(4):
                grf[:,ii] = np.matmul(force_vec[:,4*ii:4*ii+4],
                                      self.prog.GetSolution(self.beta)[4*ii:4*ii+4])
            pts = forwardKinTerrainPoints(self.rtree, kinsol, self.lfoot, self.rfoot)
            cop_x = np.matmul(pts[0], grf[2])/np.sum(grf[2])
            cop_y = np.matmul(pts[1], grf[2])/np.sum(grf[2])
            m = H[0,0]
            r_ddot = self.prog.GetSolution(self.qddot)[:3]
            cop_x2 = q[0] - (q[2]*r_ddot[0])/(r_ddot[2] + 9.81)
            cop_y2 = q[1] - (q[2]*r_ddot[1])/(r_ddot[2] + 9.81)
            qdd = (qd[:3] - self.last_v)/0.0005
            cop_x3 = q[0] - (q[2]*qdd[0])/(qdd[2] + 9.81)
            cop_y3 = q[1] - (q[2]*qdd[1])/(qdd[2] + 9.81)
            self.last_v = qd[:3]
            # Draw COM
            self.lcmgl.glColor3f(0, 0, 1)
            self.lcmgl.sphere(com[0], com[1], 0, 0.01, 20, 20)
            # Draw COP
            self.lcmgl.glColor3f(0, 1, 0)
            self.lcmgl.sphere(cop_x, cop_y, 0, 0.01, 20, 20);
            self.lcmgl.glColor3f(0, 1, 1)
            self.lcmgl.sphere(cop_x2, cop_y2, 0, 0.01, 20, 20);
            self.lcmgl.glColor3f(1, 0, 0)
            self.lcmgl.sphere(cop_x3, cop_y3, 0, 0.01, 20, 20);
            # Draw support polygon
            self.lcmgl.glColor4f(1, 0, 0, 0.5)
            self.lcmgl.glLineWidth(3)
            self.lcmgl.glBegin(0x0001)
            self.lcmgl.glVertex3d(pts[0,0], pts[1,0], pts[2,0])
            self.lcmgl.glVertex3d(pts[0,1], pts[1,1], pts[2,1])

            self.lcmgl.glVertex3d(pts[0,1], pts[1,1], pts[2,1])
            self.lcmgl.glVertex3d(pts[0,3], pts[1,3], pts[2,3])

            self.lcmgl.glVertex3d(pts[0,3], pts[1,3], pts[2,3])
            self.lcmgl.glVertex3d(pts[0,2], pts[1,2], pts[2,2])

            self.lcmgl.glVertex3d(pts[0,2], pts[1,2], pts[2,2])
            self.lcmgl.glVertex3d(pts[0,0], pts[1,0], pts[2,0])
            self.lcmgl.glEnd()
            self.lcmgl.switch_buffer()

        # Print the current time, if requested,
        # as an indicator of how far simulation has
        # progressed.
        if (self.sim and self.print_period and
            context.get_time() - self.last_print_time >= self.print_period):
            print "t: ", context.get_time()
            self.last_print_time = context.get_time()
            # print qddot_des
            for ii in range(4):
                print force_vec[:,4*ii:4*ii+4].dot(self.prog.GetSolution(self.beta)[4*ii:4*ii+4])

        if (self.sim and self.cost_pub):
            msgCost = self.packLcmCost(utime, self.prog.GetSolution(self.qddot), qddot_des,
                    self.prog.GetSolution(self.u), self.prog.GetSolution(self.eps))
            lc.publish("CASSIE_COSTS", msgCost.encode())

            empty = np.zeros(self.nu)
            msgCommand = self.packLcmCommand(utime, self.prog.GetSolution(self.u),
                    empty, empty, empty, empty, empty, empty, empty)
            lc.publish("CASSIE_COMMAND", msgCommand.encode())
