#!/usr/bin/env python

import numpy as np
import cvxpy as cp
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import os
import csv
import pandas as pd
import qpsolvers as qp
from scipy.sparse import csc_matrix

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

class DesiredTorques:
    def __init__(self):
        self.tau_d = np.zeros(7)
        self.tau_d_sub = rospy.Subscriber('cartesian_impedance_example_controller/tau_d', Float64MultiArray, self.tau_d_callback)

    def tau_d_callback(self, msg):
        self.tau_d = np.array(msg.data)


class CbfQp:
    def __init__(self, cbf_system):
        self.udim = 7  # Number of control inputs for Panda robot

        self.cbf_system = cbf_system

        self.weight_input = np.eye(self.udim) * 0.3
        self.weight_slack = 1e-3
        self.H = None
        self.slack_H = None

        self.A = None
        self.b = None

        self.cbf_gamma = 20.0

        self.u_max = np.array([87, 87, 87, 87, 12, 12, 12]) # From the Franka Robotics website

    def cbf_qp(self, u_ref):
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

        self.A = -lg_h_prime
        self.b = lf_h_prime + self.cbf_gamma * h_prime
        self.b = np.atleast_2d(self.b)[0]

        u = cp.Variable(self.udim)

        objective = cp.Minimize((1/2)*cp.quad_form(u, self.H) - (self.H @ u_ref).T @ u)

        constraints = [u <= self.u_max, self.A @ u <= self.b]

        problem = cp.Problem(objective, constraints)
    
        problem.solve(max_iter=100)

        if problem.status != 'infeasible':
            u = u.value
            feas = 1
        else:
            u = None
            feas = -1

        return u, slack, h_prime, feas
    
def shutdown_callback(event):
    rospy.loginfo("Shutting down and saving data to CSV files...")
    
    # Convert lists to DataFrames
    u_ref_df = pd.DataFrame(u_ref_list, columns=[f'Joint {i+1}' for i in range(7)])
    u_df = pd.DataFrame(u_list, columns=[f'Joint {i+1}' for i in range(7)])

    # Save DataFrames to CSV files
    u_ref_df.to_csv('src/franka_ros/franka_example_controllers/results/tau_nom.csv', index=False)
    u_df.to_csv('src/franka_ros/franka_example_controllers/results/tau_optim.csv', index=False)

    rospy.signal_shutdown("Time limit reached")


if __name__ == '__main__':
    rospy.init_node('panda_cbf_qp', anonymous=True)

    panda_dynamics_model = PandaDynamicsModel()
    panda_joint_states = PandaJointStates()

    desired_torques = DesiredTorques()

    control_input_pub = rospy.Publisher('tau_star', Float64MultiArray, queue_size=10)

    u_ref_list = []
    u_list = []

    def save_to_csv(filename, data):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            for row in data:
                writer.writerow(row)

    rate = rospy.Rate(1000)
    
    rospy.Timer(rospy.Duration(30), shutdown_callback, oneshot=True)

    while not rospy.is_shutdown():

        q = panda_joint_states.positions
        dq = panda_joint_states.velocities
        D = panda_dynamics_model.inertia_matrix
        C = panda_dynamics_model.coriolis
        g = panda_dynamics_model.gravity_vector

        u_ref = desired_torques.tau_d

        alpha = 5.0

        cbf_system = CBF(q, dq, D, C, g, alpha)
        qp_solver = CbfQp(cbf_system)

        start_time = rospy.Time.now()
        u, slack, h, feas = qp_solver.cbf_qp(u_ref)
        solve_time = rospy.Time.now() - start_time

        rospy.loginfo("Nominal Control Input: %s", u_ref)
        rospy.loginfo("Optimal Control Input: %s", u)
        rospy.loginfo("CBF Value: %s", h)
        rospy.loginfo("Solve Time: %s seconds", solve_time.to_sec())

        if u is not None:
            u_ref_list.append(u_ref.tolist())
            u_list.append(u.tolist())

        control_input_msg = Float64MultiArray()
        control_input_msg.data = u

        control_input_pub.publish(control_input_msg)


        rate.sleep()