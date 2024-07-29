
import numpy as np
import cvxpy as cp

class CbfQp:
    """
    This is the implementation of the CBF-QP method. The optimization problem is:

            min (u-u_ref).T * H * (u-u_ref) + p * delta**2
            s.t. L_f B(x) + L_g B(x) * u + gamma * B(x) >= 0  ---> CBF constraint

    Input:
    :param  system  :   The dynamic system of interest, containing CBF and their Lie derivatives
    :param  x       :   The current state x
    :param  u_ref   :   The reference control input
    :param  slack   :   The slack activated or not, 1 -> activate while 0 -> not activate
    :param  verbose :   Show the optimization log or not
    """
    def __init__(self, system, cbf_gamma=5.0, weight_input=None, weight_slack=0.01, u_max=None, u_min=None):
        self.system = syste
import numpy as np
import cvxpy as cp

class CbfQp:
    """
    This is the implementation of the CBF-QP method. The optimization problem is:

            min (u-u_ref).T * H * (u-u_ref) + p * delta**2
            s.t. L_f B(x) + L_g B(x) * u + gamma * B(x) >= 0  ---> CBF constraint

    Input:
    :param  system  :   The dynamic system of interest, containing CBF and their Lie derivatives
    :param  x       :   The current state x
    :param  u_ref   :   The reference control input
    :param  slack   :   The slack activated or not, 1 -> activate while 0 -> not activate
    :param  verbose :   Show the optimization log or not
    """
    def __init__(self, system, cbf_gamma=5.0, weight_input=None, weight_slack=0.01, u_max=None, u_min=None):
        self.system = system
        self.udim = system.udim

        self.cbf = system.cbf
        self.lf_cbf = system.lf_cbf
        self.lg_cbf = system.lg_cbf

        self.cbf_gamma = cbf_gamma
        self.weight_input = np.eye(self.udim) if weight_input is None else weight_input
        self.weight_slack = weight_slack
        self.u_max = np.ones(self.udim) * np.inf if u_max is None else u_max
        self.u_min = np.ones(self.udim) * -np.inf if u_min is None else u_min

        self.H = None
        self.slack_H = None
        self.A = None
        self.b = None
        self.with_slack = None

    def cbf_qp(self, x, u_ref=None, with_slack=1, verbose=0):
        inf = np.inf
        self.with_slack = with_slack

        slack = None
        if u_ref is None:
            u_ref = np.zeros(self.udim)
        else:
            if u_ref.shape != (self.udim,):
                raise ValueError(f'u_ref should have the shape size (u_dim,), now it is {u_ref.shape}')

        # Read the weight input and build up the matrix H in the cost function
        if self.weight_input.shape == (1, 1):
            self.H = self.weight_input * np.eye(self.udim)
        elif self.weight_input.shape == (self.udim, 1):
            self.H = np.diag(self.weight_input)
        elif self.weight_input.shape == (self.udim, self.udim):
            self.H = np.copy(self.weight_input)
        else:
            self.H = np.eye(self.udim)

        h = self.cbf(x)
        lf_B = self.lf_cbf(x)
        lg_B = self.lg_cbf(x)

        if self.with_slack:
            lg_B = np.hstack([lg_B, np.zeros((lg_B.shape[0], 1))])
            self.A = lg_B
            self.b = lf_B + self.cbf_gamma * h
            self.b = np.atleast_2d(self.b)[0]

            u_min = np.hstack([self.u_min, -inf * np.ones(1)])
            u_max = np.hstack([self.u_max, inf * np.ones(1)])

            u = cp.Variable(self.udim + 1)
            self.slack_H = np.hstack([self.H, np.zeros((self.H.shape[0], 1))])
            self.slack_H = np.vstack([self.slack_H, np.hstack([np.zeros((1, self.H.shape[1])), self.weight_slack])])

            u_ref = np.hstack([u_ref, np.zeros(1)])
            objective = cp.Minimize(0.5 * cp.quad_form(u, self.slack_H) - u_ref.T @ self.slack_H @ u)

            # Constraints: A * u <= b and u_min, u_max
            constraints = [u_min <= u, u <= u_max, self.A @ u <= self.b]

            problem = cp.Problem(objective, constraints)
            problem.solve()

            if problem.status != cp.OPTIMAL:
                raise ValueError("QP problem is not feasible")

            u_opt = u.value[:-1]
            slack_opt = u.value[-1]
        else:
            self.A = lg_B
            self.b = lf_B + self.cbf_gamma * h
            self.b = np.atleast_2d(self.b)[0]

            u = cp.Variable(self.udim)
            objective = cp.Minimize(0.5 * cp.quad_form(u, self.H) - u_ref.T @ self.H @ u)
            constraints = [self.u_min <= u, u <= self.u_max, self.A @ u <= self.b]

            problem = cp.Problem(objective, constraints)
            problem.solve()

            if problem.status != cp.OPTIMAL:
                raise ValueError("QP problem is not feasible")

            u_opt = u.value
            slack_opt = None

        return u_opt, slack_opt, h
m
        self.udim = system.udim

        self.cbf = system.cbf
        self.lf_cbf = system.lf_cbf
        self.lg_cbf = system.lg_cbf

        self.cbf_gamma = cbf_gamma
        self.weight_input = np.eye(self.udim) if weight_input is None else weight_input
        self.weight_slack = weight_slack
        self.u_max = np.ones(self.udim) * np.inf if u_max is None else u_max
        self.u_min = np.ones(self.udim) * -np.inf if u_min is None else u_min

        self.H = None
        self.slack_H = None
        self.A = None
        self.b = None
        self.with_slack = None

    def cbf_qp(self, x, u_ref=None, with_slack=1, verbose=0):
        inf = np.inf
        self.with_slack = with_slack

        slack = None
        if u_ref is None:
            u_ref = np.zeros(self.udim)
        else:
            if u_ref.shape != (self.udim,):
                raise ValueError(f'u_ref should have the shape size (u_dim,), now it is {u_ref.shape}')

        # Read the weight input and build up the matrix H in the cost function
        if self.weight_input.shape == (1, 1):
            self.H = self.weight_input * np.eye(self.udim)
        elif self.weight_input.shape == (self.udim, 1):
            self.H = np.diag(self.weight_input)
        elif self.weight_input.shape == (self.udim, self.udim):
            self.H = np.copy(self.weight_input)
        else:
            self.H = np.eye(self.udim)

        h = self.cbf(x)
        lf_B = self.lf_cbf(x)
        lg_B = self.lg_cbf(x)

        if self.with_slack:
            lg_B = np.hstack([lg_B, np.zeros((lg_B.shape[0], 1))])
            self.A = lg_B
            self.b = lf_B + self.cbf_gamma * h
            self.b = np.atleast_2d(self.b)[0]

            u_min = np.hstack([self.u_min, -inf * np.ones(1)])
            u_max = np.hstack([self.u_max, inf * np.ones(1)])

            u = cp.Variable(self.udim + 1)
            self.slack_H = np.hstack([self.H, np.zeros((self.H.shape[0], 1))])
            self.slack_H = np.vstack([self.slack_H, np.hstack([np.zeros((1, self.H.shape[1])), self.weight_slack])])

            u_ref = np.hstack([u_ref, np.zeros(1)])
            objective = cp.Minimize(0.5 * cp.quad_form(u, self.slack_H) - u_ref.T @ self.slack_H @ u)

            # Constraints: A * u <= b and u_min, u_max
            constraints = [u_min <= u, u <= u_max, self.A @ u <= self.b]

            problem = cp.Problem(objective, constraints)
            problem.solve()

            if problem.status != cp.OPTIMAL:
                raise ValueError("QP problem is not feasible")

            u_opt = u.value[:-1]
            slack_opt = u.value[-1]
        else:
            self.A = lg_B
            self.b = lf_B + self.cbf_gamma * h
            self.b = np.atleast_2d(self.b)[0]

            u = cp.Variable(self.udim)
            objective = cp.Minimize(0.5 * cp.quad_form(u, self.H) - u_ref.T @ self.H @ u)
            constraints = [self.u_min <= u, u <= self.u_max, self.A @ u <= self.b]

            problem = cp.Problem(objective, constraints)
            problem.solve()

            if problem.status != cp.OPTIMAL:
                raise ValueError("QP problem is not feasible")

            u_opt = u.value
            slack_opt = None

        return u_opt, slack_opt, h
