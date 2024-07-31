#!/usr/bin/env python

import roboticstoolbox as rtb
import sympy as sp
import numpy as np

# Define DH parameters for each joint
panda = rtb.models.DH.Panda()

# Define symbolic variables for joint angles and velocities using SymPy
q = list(sp.symbols('q1:8'))
dq = list(sp.symbols('dq1:8'))

# Compute the forward kinematics using the DH model
T = panda.fkine(q)

rotation = T.R

# Define the reference vector in the x-direction
e_x = sp.Matrix([1, 0, 0])

e_z = sp.Matrix([0, 0, 1])

# Perform the matrix multiplication e^T * R * e
orient_vec = e_z.T * T.R * e_x

# Define theta_max and compute cos(theta_max)
theta_max = sp.pi / 4
cos_theta_max = sp.cos(theta_max)

# Ensure result and cos_theta_max are compatible for subtraction
# Create a matrix with the same shape as `result` filled with `cos_theta_max`
cos_matrix = sp.Matrix(orient_vec.shape[0], orient_vec.shape[1], lambda i, j: cos_theta_max)

# Define the CBF constraint h(x)
h = orient_vec - cos_matrix

# Define the robot state vector x = [theta, theta_dot]^T
x = q + dq

# Differentiate the result_simplified with respect to the robot states
dh_dx = sp.Matrix([sp.diff(h, var) for var in x]).T

f_x = dq + [0 for _ in range(7)]

L_f_x = dh_dx @ sp.Matrix(f_x)

alpha = sp.symbols("alpha")

h_prime = L_f_x + alpha * h

dh_prime_dx = sp.Matrix([sp.diff(L_f_x, var) for var in x]).T + alpha*dh_dx

vars = [h, dh_dx, h_prime, dh_prime_dx]

def export_result(var):
    with open(f'constraints/{var}.txt', 'w') as file:
        file.write(str(var))

for var in vars:
    export_result(var)