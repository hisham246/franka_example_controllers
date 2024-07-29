#!/usr/bin/env python

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import numpy as np

from example_robot_data import load

robot = load('panda')

# urdf_model_path = "src/franka_ros/franka_description/robots/panda/franka_panda.urdf"

# Load the robot model
# robot = RobotWrapper.BuildFromURDF(urdf_model_path)

model = robot.model
data = model.createData()

# Generate a random configuration
q = pin.randomConfiguration(model)

# Compute the inertia matrix at the given configuration
M = pin.crba(model, data, q)

# # # Number of joints
nq = model.nq

print(M)

# print("Joint Names and Types:")
# for i, joint in enumerate(model.joints):
#     print(f"Joint {i}: {joint}, Type: {joint.shortname()}")

# # Initialize the tensor to store the partial derivatives of the inertia matrix
# dM_dq = np.zeros((nq, nq, nq))
# # Compute the partial derivatives of the inertia matrix with respect to q
# for i in range(nq):
#     # Perturb the i-th joint position
#     dq = np.zeros(nq)
#     dq[i] = 1e-8  # small perturbation
#     q_plus = pin.integrate(model, q, dq)
#     q_minus = pin.integrate(model, q, -dq)
    
#     # Compute the inertia matrix at the perturbed configurations
#     M_plus = pin.crba(model, data, q_plus)
#     M_minus = pin.crba(model, data, q_minus)
    
#     # Compute the numerical derivative
#     dM_dq[:, :, i] = (M_plus - M_minus) / (2 * 1e-8)

# # Print the inertia matrix and its derivatives
# print("Inertia Matrix (M):")
# print(M)

# print("Partial Derivatives of the Inertia Matrix with respect to q (dM/dq):")
# print(dM_dq)
