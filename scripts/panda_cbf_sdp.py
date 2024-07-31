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

class Jacobian:
    def __init__(self):
        self.jacobian = np.zeros((6,7))

        self.jacobian_sub = rospy.Subscriber('cartesian_impedance_example_controller/jacobian', Float64MultiArray, self.jacobian_callback)

    def jacobian_callback(self, msg):
        self.jacobian = np.array(msg.data).reshape((6, 7))

class Error:
    def __init__(self):
        self.error = np.zeros(6)
        self.error_sub = rospy.Subscriber('cartesian_impedance_example_controller/error', Float64MultiArray, self.error_callback)

    def error_callback(self, msg):
        self.error = np.array(msg.data).reshape(6,1)

class TauNullspace:
    def __init__(self):
        self.tau_nullspace = np.zeros(7)
        self.tau_nullspace_sub = rospy.Subscriber('cartesian_impedance_example_controller/tau_nullspace', Float64MultiArray, self.tau_nullspace_callback)

    def tau_nullspace_callback(self, msg):
        self.tau_nullspace = np.array(msg.data).reshape(7,1)

class CbfSdp:
    def __init__(self, cbf_system):
        self.udim = 7  # Number of control inputs for Panda robot

        self.cbf_system = cbf_system

        self.weight_input = np.eye(self.udim)
        self.weight_slack = 2e-2

        self.cbf_gamma = 5.0

        self.u_max = np.array([87, 87, 87, 87, 12, 12, 12])  # From the Franka Robotics website
    
    # def jacobian_transpose(self, J):
    #     J_T = 

    def cbf_sdp(self, J, error, dq, tau_nullspace, with_slack=1):
        inf = np.inf

        # Define the variable K_p and K_d as diagonal matrices
        K_p_diag = cp.Variable(6, nonneg=True)
        K_p = cp.diag(K_p_diag)

        # K_d_diag = cp.Variable(6, nonneg=True)
        # K_d = cp.diag(K_d_diag)

        # Define the control input u
        u = J.T @ (-K_p @ error - 2*cp.sqrt(K_p) @ (J @ dq)) + tau_nullspace

        # Define the objective function
        objective = cp.Minimize(cp.sum(K_p_diag))  # Example objective

        # CBF constraints
        h_prime = self.cbf_system.h_prime_x()
        lf_h_prime = self.cbf_system.dh_prime_dx() @ self.cbf_system.f_x()
        lg_h_prime = self.cbf_system.dh_prime_dx() @ self.cbf_system.g_x()

        constraints = [
            lf_h_prime + lg_h_prime @ u >= -self.cbf_gamma * h_prime
        ]

        if with_slack:
            slack = cp.Variable()
            constraints += [slack >= 0]
            constraints += [lf_h_prime + lg_h_prime @ u + slack >= -self.cbf_gamma * h_prime]
            objective += self.weight_slack * slack

        # Define and solve the problem
        prob = cp.Problem(objective, constraints)
        prob.solve()

        return K_p.value

if __name__ == '__main__':
    rospy.init_node('panda_cbf_sdp', anonymous=True)

    panda_dynamics_model = PandaDynamicsModel()
    panda_joint_states = PandaJointStates()
    jacobian = Jacobian()
    error = Error()
    tau_nullspace = TauNullspace()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        q = panda_joint_states.positions
        dq = panda_joint_states.velocities
        D = panda_dynamics_model.inertia_matrix
        C = panda_dynamics_model.coriolis
        g = panda_dynamics_model.gravity_vector
        alpha = 2.0

        # cbf_system = CBF(q, dq, D, C, g, alpha)
        # sdp_solver = CbfSdp(cbf_system)

        J = jacobian.jacobian
        error_val = error.error
        tau_nullspace_val = tau_nullspace.tau_nullspace

        # K_p = sdp_solver.cbf_sdp(J, error_val, np.array(dq).reshape(7,1), tau_nullspace_val)

        # rospy.loginfo("Optimal K_p: %s", K_p)
        # rospy.loginfo("Optimal K_d: %s", K_d)

        rospy.loginfo("Jacobian: %s", J.shape)
        rospy.loginfo("Jacobian transpose: %s", J.T.shape)
        # rospy.loginfo("Stiffness: %s", K_p.shape)
        # rospy.loginfo("Damping: %s", K_d.shape)
        # rospy.loginfo("Pose error: %s", x_tilde.shape)
        # rospy.loginfo("Joint velocities: %s", dq_array.shape)
        rospy.loginfo("Nullspace tau: %s", tau_nullspace_val.shape)

        rate.sleep()

# import cvxpy as cp
# import numpy as np

# # Define dimensions
# n = 6  # Dimension of K_p (assumed 6x6 for task space)
# m = 7  # Number of control inputs

# # Placeholder values for problem data, replace with actual values
# A = np.random.randn(n, n)
# L_f_h = np.random.randn()
# L_g_h = np.random.randn(1, m)
# J = np.random.randn(n, m)
# x_tilde = np.random.randn(n, 1)
# x_tilde_dot = np.random.randn(n, 1)
# tau_nullspace = np.random.randn(m, 1)
# gamma = 1.0
# h = np.random.randn()

# # Define the variable K_p as a diagonal matrix
# K_p_diag = cp.Variable(n)
# K_p = cp.diag(K_p_diag)

# K_d_diag = cp.Variable(n)
# K_d = cp.diag(K_d_diag)

# # Define the slack variable
# slack = cp.Variable()

# # Define the control input u
# u = J.T @ (-K_p @ x_tilde - K_d @ x_tilde_dot) + tau_nullspace

# # Define the objective function
# objective = cp.Minimize(cp.trace(A @ K_p) + 1e3 * slack)

# # Add the CBF constraint with slack variable
# constraints = [
#     L_f_h + L_g_h @ u + slack >= -gamma * h
# ]

# # Ensure K_p is positive definite
# constraints += [K_p_diag >= 0]

# # Ensure slack is non-negative
# constraints += [slack >= 0]

# # Define and solve the problem
# prob = cp.Problem(objective, constraints)
# prob.solve()

# # Output the results
# print("The optimal value is", prob.value)
# print("A solution K_p is")
# print(K_p.value)
# print("The K_d diagonal elements are")
# print(K_d_diag.value)
# print("The slack variable is")
# print(slack.value)