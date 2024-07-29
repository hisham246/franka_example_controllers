import numpy as np
import sympy as sp

class PandaSystem:
    """
    Define the symbolic dynamic:    dx = f(x) + g(x) * u

    """
    def __init__(self, params):
        self.D = params['D']
        self.C = params['C']
        self.g = params['g']
        self.R = params['R']
        self.theta_max = params['theta_max']

        q1, q2, q3, q4, q5, q6, q7 = sp.symbols('q1 q2 q3 q4 q5 q6 q7')
        dq1, dq2, dq3, dq4, dq5, dq6, dq7 = sp.symbols('dq1 dq2 dq3 dq4 dq5 dq6 dq7')
        self.x = sp.Matrix([q1, q2, q3, q4, q5, q6, q7, dq1, dq2, dq3, dq4, dq5, dq6, dq7])

        # Define the symbolic expression for system dynamics and CBF
        self.f, self.g = self.panda_dynamics()
        self.cbf = self.define_cbf()

        if 'udim' in params.keys():
            self.udim = params['udim']
        else:
            self.udim = 7

    def panda_dynamics(self):
        D = self.D(self.x[:7])
        C = self.C(self.x[:7], self.x[7:])
        g = self.g(self.x[:7])

        # f, g both column vector
        f = sp.Matrix.vstack(self.x[7:], -D.inv() * (C + g))
        g_matrix = sp.Matrix.vstack(sp.zeros(7, 7), D.inv())
        return f, g_matrix
    
    def define_cbf(self):
        e = sp.Matrix([0, 0, 1])
        R = self.R(self.x[:7])
        cos_theta_max = sp.cos(self.theta_max)

        orient_vec = sp.simplify(e.T * R * e)
        cos_matrix = sp.Matrix(orient_vec.shape[0], orient_vec.shape[1], lambda i, j: cos_theta_max)

        cbf = orient_vec - cos_matrix
        return cbf