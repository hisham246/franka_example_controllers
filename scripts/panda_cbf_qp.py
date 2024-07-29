#!/usr/bin/env python

import numpy as np
import cvxpy as cp
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from cbf_system import CBF

class PandaDynamicsModel:
    def __init__(self):
        self.coriolis = np.zeros(7)
        self.inertia_matrix = np.zeros((7, 7))
        self.gravity_vector = np.zeros(7)

        self.inertia_matrix_sub = rospy.Subscriber('cartesian_impedance_example_controller/inertia_matrix', Float64MultiArray, self.inertia_matrix_callback)
        self.coriolis_sub = rospy.Subscriber('cartesian_impedance_example_controller/coriolis', Float64MultiArray, self.coriolis_callback)
        self.gravity_vector_sub = rospy.Subscriber('cartesian_impedance_example_controller/gravity', Float64MultiArray, self.gravity_callback)

    def inertia_matrix_callback(self, msg):
        self.inertia_matrix = np.array(msg.data).reshape((7, 7))

    def coriolis_callback(self, msg):
        self.coriolis = np.array(msg.data)

    def gravity_callback(self, msg):
        self.gravity_vector = np.array(msg.data)

class PandaJointStates:
    def __init__(self):
        self.positions = np.zeros(7)
        self.velocities = np.zeros(7)

        self.joint_states_sub = rospy.Subscriber('franka_state_controller/joint_states', JointState, self.joint_states_callback)

    def joint_states_callback(self, msg):
        self.positions = np.array(msg.position)
        self.velocities = np.array(msg.velocity)

class CbfQp:
    def __init__(self, cbf_system):
        self.udim = 7  # Number of control inputs for Panda robot

        self.cbf_system = cbf_system

        self.weight_input = np.eye(self.udim)
        self.weight_slack = 2e-2
        self.H = None
        self.slack_H = None

        self.A = None
        self.b = None

        self.cbf_gamma = 5.0

        self.u_max = np.array([87, 87, 87, 87, 12, 12, 12]) # From the Franka Robotics website

    def cbf_qp(self, x, u_ref=None, with_slack=1):
        inf = np.inf
        slack = None
        if u_ref is None:
            u_ref = np.zeros(self.udim)
        else:
            if u_ref.shape != (self.udim,):
                raise ValueError(f'u_ref should have the shape size (u_dim,), now it is {u_ref.shape}')

        self.H = self.weight_input

        h_prime = self.cbf_system.h_prime_x()
        lf_h_prime = self.cbf_system.dh_prime_dx() @ self.cbf_system.f_x()
        lg_h_prime = self.cbf_system.dh_prime_dx() @ self.cbf_system.g_x()

        if with_slack:
            lg_h_prime = np.hstack((lg_h_prime, np.zeros((lg_h_prime.shape[0], 1))))

            self.A = lg_h_prime
            self.b = lf_h_prime + self.cbf_gamma * h_prime

            self.b = np.atleast_2d(self.b)[0]

            u_max = np.hstack((self.u_max, inf * np.ones(1)))

            u = cp.Variable(self.udim + 1)

            self.slack_H = np.hstack((self.H, np.zeros((self.H.shape[0], 1))))
            self.slack_H = np.vstack((self.slack_H, np.hstack((np.zeros((1, self.H.shape[0])), self.weight_slack * np.ones((1, 1))))))

            u_ref = np.hstack((u_ref, np.zeros(1)))
            objective = cp.Minimize((1/2) * cp.quad_form(u, self.slack_H) - (self.slack_H @ u_ref).T @ u)

            constraints = [u <= u_max, self.A @ u <= self.b]

            problem = cp.Problem(objective, constraints)

            problem.solve()

            if problem.status != 'infeasible':
                slack = u.value[-1]
                u = u.value[:self.udim]
                feas = 1
            else:
                u = None
                slack = None
                feas = -1

        else:
            self.A = -lg_h_prime
            self.b = lf_h_prime + self.cbf_gamma * h_prime
            self.b = np.atleast_2d(self.b)[0]

            u = cp.Variable(self.udim)

            objective = cp.Minimize((1/2)*cp.quad_form(u, self.H) - (self.H @ u_ref).T @ u)

            constraints = [u <= self.u_max, self.A @ x <= self.b]

            problem = cp.Problem(objective, constraints)

            problem.solve()

            if problem.status != 'infeasible':
                u = u.value
                feas = 1
            else:
                u = None
                feas = -1

        return u, slack, h_prime, feas

if __name__ == '__main__':
    rospy.init_node('panda_cbf_qp', anonymous=True)

    panda_dynamics_model = PandaDynamicsModel()
    panda_joint_states = PandaJointStates()


    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        q = panda_joint_states.positions
        dq = panda_joint_states.velocities
        D = panda_dynamics_model.inertia_matrix
        C = panda_dynamics_model.coriolis
        g = panda_dynamics_model.gravity_vector
        alpha = 2.0

        cbf_system = CBF(q, dq, D, C, g, alpha)
        qp_solver = CbfQp(cbf_system)

        x = np.hstack((q, dq))
        u_ref = 10 * np.ones(7)

        u, slack, h, feas = qp_solver.cbf_qp(x, u_ref)

        rospy.loginfo("Optimal Control Input: %s", u)
        rospy.loginfo("CBF Value: %s", h)

        rate.sleep()