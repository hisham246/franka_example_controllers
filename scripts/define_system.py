import sympy as sp
from sympy.utilities.lambdify import lambdify
import numpy as np

class ControlAffineSystem:
    """
    This class defines the dynamic of the control affine system:

            dx = f(x) + g(x) * u
    f -> 2D column-wise vector
    g -> 2D column-wise vector

    This class also includes the control barrier function (cbf) in a symbolic way.

    This class has the following methods:

    - Compute the Lie derivative of CBF w.r.t. f(x) and g(x)
    """

    def __init__(self, system, alpha=1.0):
        self.x = system.x  # column-wise vector
        self.xdim = self.x.shape[0]
        self.udim = system.udim

        self.f = None
        self.f_symbolic = None

        self.g = None
        self.g_symbolic = None

        self.cbf = None
        self.cbf_symbolic = None

        self.alpha = alpha

        # Lie derivative of cbf w.r.t f as a function
        self.lf_cbf = None
        self.lf_cbf_symbolic = None

        # Lie derivative of cbf w.r.t g as a function
        self.lg_cbf = None
        self.lg_cbf_symbolic = None

        # Lie derivative of h'(x) w.r.t f as a function
        self.lf_cbf_prime = None
        self.lf_cbf_prime_symbolic = None

        # Lie derivative of h'(x) w.r.t g as a function
        self.lg_cbf_prime = None
        self.lg_cbf_prime_symbolic = None

        self.define_system(system)
        self.define_cbf(system.cbf)
        self.lie_derivatives_calculator()

    def define_system(self, dynamic_system_class):
        self.f_symbolic = dynamic_system_class.f
        self.f = lambdify(np.array(self.x.T), self.f_symbolic, 'numpy')
        if self.f(np.ones(self.xdim)).shape != (self.xdim, 1):
            raise ValueError(f'The output of f(x) should be (xdim, 1), now it is {self.f(np.ones(self.xdim)).shape}')

        self.g_symbolic = dynamic_system_class.g
        self.g = lambdify(np.array(self.x.T), self.g_symbolic, 'numpy')
        if self.g(np.ones(self.xdim)).shape != (self.xdim, 1):
            raise ValueError(f'The output of g(x) should be (xdim, 1), now it is {self.g(np.ones(self.xdim)).shape}')
        
    def define_cbf(self, cbf):
        """
        Define the symbolic control barrier function
        :param cbf:
        :return:
        """
        self.cbf_symbolic = cbf
        self.cbf = lambdify(np.array(self.x.T), self.cbf_symbolic, 'numpy')

    def lie_derivatives_calculator(self):
        """
        Compute the Lie derivatives of CBF w.r.t to x
        :return:
        """
        dx_cbf_symbolic = sp.Matrix([self.cbf_symbolic]).jacobian(self.x)

        self.lf_cbf_symbolic = dx_cbf_symbolic * self.f_symbolic
        self.lg_cbf_symbolic = dx_cbf_symbolic * self.g_symbolic

        self.lf_cbf = lambdify(np.array(self.x.T), self.lf_cbf_symbolic, 'numpy')
        self.lg_cbf = lambdify(np.array(self.x.T), self.lg_cbf_symbolic, 'numpy')

        # Define h_prime(x) = Lf_h + alpha * h
        h_prime_symbolic = self.lf_cbf_symbolic + self.alpha * self.cbf_symbolic
        dx_h_prime_symbolic = h_prime_symbolic.jacobian(self.x)

        self.lf_cbf_prime_symbolic = dx_h_prime_symbolic * self.f_symbolic
        self.lg_cbf_prime_symbolic = dx_h_prime_symbolic * self.g_symbolic

        self.lf_cbf_prime = lambdify(np.array(self.x.T), self.lf_cbf_prime_symbolic, 'numpy')
        self.lg_cbf_prime = lambdify(np.array(self.x.T), self.lg_cbf_prime_symbolic, 'numpy')


# import sympy as sp
# from sympy.utilities.lambdify import lambdify
# import numpy as np

# class ControlAffineSystem:
#     """
#         This class defines the dynamic of the control affine system:

#                 dx = f(x) + g(x) * u
#         f -> 2D column-wise vector
#         g -> 2D column-wise vector

#         This class also includes the control barrier function (cbf) in a symbolic way.

#         This class has the following methods:

#         - Compute the Lie derivative of CBF w.r.t. f(x) and g(x):
#     """

#     def __init__(self, system):
#         self.x = system.x  # column-wise vector
#         self.xdim = system.x.shape[0]
#         self.xdim = self.x.shape[0]
#         self.udim = system.udim

#         self.f = None
#         self.f_symbolic = None

#         self.g = None
#         self.g_symbolic = None

#         self.cbf = None
#         self.cbf_symbolic = None

#         # Lie derivative of cbf w.r.t f as a function
#         self.lf_cbf = None
#         self.lf_cbf_symbolic = None

#         # Lie derivative of cbf w.r.t g as a function
#         self.lg_cbf = None
#         self.lg_cbf_symbolic = None

#         self.define_system(system)
#         self.define_cbf(system.cbf)
#         self.lie_derivatives_calculator()

#     def define_system(self, dynamic_system_class):
#         self.f_symbolic = dynamic_system_class.f
#         self.f = lambdify(np.array(self.x.T), self.f_symbolic, 'numpy')
#         if self.f(np.ones(self.xdim)).shape != (self.xdim, 1):
#             raise ValueError(f'The output of f(x) should be (xdim, 1), now it is {self.f(np.ones(self.xdim)).shape}')

#         self.g_symbolic = dynamic_system_class.g
#         self.g = lambdify(np.array(self.x.T), self.g_symbolic, 'numpy')
#         if self.g(np.ones(self.xdim)).shape != (self.xdim, 1):
#             raise ValueError(f'The output of g(x) should be (xdim, 1), now it is {self.g(np.ones(self.xdim)).shape}')
        
#     def define_cbf(self, cbf):
#         """
#         Define the symbolic control barrier function
#         :param cbf:
#         :return:
#         """
#         self.cbf_symbolic = cbf
#         self.cbf = lambdify(np.array(self.x.T), self.cbf_symbolic, 'numpy')

#     def lie_derivatives_calculator(self):
#         """
#         Compute the Lie derivatives of CBF w.r.t to x
#         :return:
#         """
#         dx_cbf_symbolic = sp.Matrix([self.cbf_symbolic]).jacobian(self.x)

#         self.lf_cbf_symbolic = dx_cbf_symbolic * self.f_symbolic
#         self.lg_cbf_symbolic = dx_cbf_symbolic * self.g_symbolic

#         self.lf_cbf = lambdify(np.array(self.x.T), self.lf_cbf_symbolic, 'numpy')
#         self.lg_cbf = lambdify(np.array(self.x.T), self.lg_cbf_symbolic, 'numpy')

# import casadi as ca
# import numpy as np

# class ControlAffineSystem:
#     """
#     This class defines the dynamic of the control affine system:

#             dx = f(x) + g(x) * u
#     f -> 2D column-wise vector
#     g -> 2D column-wise vector

#     This class also includes the control barrier function (cbf) in a symbolic way.

#     This class has the following methods:

#     - Compute the Lie derivative of CBF w.r.t. f(x) and g(x):
#     """

#     def __init__(self, system):
#         self.x = system.x  # column-wise vector
#         self.xdim = self.x.shape[0]
#         self.udim = system.udim

#         self.f = None
#         self.f_symbolic = None

#         self.g = None
#         self.g_symbolic = None

#         self.cbf = None
#         self.cbf_symbolic = None

#         # Lie derivative of cbf w.r.t f as a function
#         self.lf_cbf = None
#         self.lf_cbf_symbolic = None

#         # Lie derivative of cbf w.r.t g as a function
#         self.lg_cbf = None
#         self.lg_cbf_symbolic = None

#         self.define_system(system)
#         self.define_cbf(system.cbf)
#         self.lie_derivatives_calculator()

#     def define_system(self, dynamic_system_class):
#         self.f_symbolic = dynamic_system_class.f
#         self.f = ca.Function('f', [self.x], [self.f_symbolic])
#         if self.f(np.ones((self.xdim, 1))).shape != (self.xdim, 1):
#             raise ValueError(f'The output of f(x) should be (xdim, 1), now it is {self.f(np.ones((self.xdim, 1))).shape}')

#         self.g_symbolic = dynamic_system_class.g
#         self.g = ca.Function('g', [self.x], [self.g_symbolic])
#         if self.g(np.ones((self.xdim, 1))).shape != (self.xdim, 1):
#             raise ValueError(f'The output of g(x) should be (xdim, 1), now it is {self.g(np.ones((self.xdim, 1))).shape}')
        
#     def define_cbf(self, cbf):
#         """
#         Define the symbolic control barrier function
#         :param cbf:
#         :return:
#         """
#         self.cbf_symbolic = cbf
#         self.cbf = ca.Function('cbf', [self.x], [self.cbf_symbolic])

#     def lie_derivatives_calculator(self):
#         """
#         Compute the Lie derivatives of CBF w.r.t to x
#         :return:
#         """
#         dx_cbf_symbolic = ca.jacobian(self.cbf_symbolic, self.x)

#         self.lf_cbf_symbolic = dx_cbf_symbolic @ self.f_symbolic
#         self.lg_cbf_symbolic = dx_cbf_symbolic @ self.g_symbolic

#         self.lf_cbf = ca.Function('lf_cbf', [self.x], [self.lf_cbf_symbolic])
#         self.lg_cbf = ca.Function('lg_cbf', [self.x], [self.lg_cbf_symbolic])
