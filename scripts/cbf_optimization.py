# from scipy.sparse import csc_matrix


        # if with_slack:
        #     lg_h_prime = np.hstack((lg_h_prime, np.zeros((lg_h_prime.shape[0], 1))))

        #     self.A = lg_h_prime
        #     self.b = lf_h_prime + self.cbf_gamma * h_prime

        #     self.b = np.atleast_2d(self.b)[0]

        #     u_max = np.hstack((self.u_max, inf * np.ones(1)))

        #     u = cp.Variable(self.udim + 1)

        #     self.slack_H = np.hstack((self.H, np.zeros((self.H.shape[0], 1))))
        #     self.slack_H = np.vstack((self.slack_H, np.hstack((np.zeros((1, self.H.shape[0])), self.weight_slack * np.ones((1, 1))))))

        #     u_ref = np.hstack((u_ref, np.zeros(1)))
        #     objective = cp.Minimize((1/2) * cp.quad_form(u, self.slack_H) - (self.slack_H @ u_ref).T @ u)

        #     constraints = [u <= u_max, self.A @ u <= self.b]

        #     problem = cp.Problem(objective, constraints)

        #     problem.solve(max_iter=10000)

        #     if problem.status != 'infeasible':
        #         slack = u.value[-1]
        #         u = u.value[:self.udim]
        #         feas = 1
        #     else:
        #         u = None
        #         slack = None
        #         feas = -1

        # else:


# class CbfQp:
#     def __init__(self, cbf_system):
#         self.udim = 7  # Number of control inputs for Panda robot

#         self.cbf_system = cbf_system

#         self.weight_input = np.eye(self.udim) * 0.3
#         self.weight_slack = 1e-3

#         self.cbf_gamma = 20.0

#         self.u_max = np.array([87, 87, 87, 87, 12, 12, 12]) # From the Franka Robotics website

#     def cbf_qp(self, u_ref, with_slack=1):
#         if u_ref is None:
#             u_ref = np.zeros(self.udim)
#         else:
#             if u_ref.shape != (self.udim,):
#                 raise ValueError(f'u_ref should have the shape size (u_dim,), now it is {u_ref.shape}')

#         H = self.weight_input
#         f = -H @ u_ref

#         h_prime = self.cbf_system.h_prime_x()
#         lf_h_prime = self.cbf_system.dh_prime_dx() @ self.cbf_system.f_x()
#         lg_h_prime = self.cbf_system.dh_prime_dx() @ self.cbf_system.g_x()

#         A = -lg_h_prime
#         b = lf_h_prime + self.cbf_gamma * h_prime

#         lb = -self.u_max
#         ub = self.u_max

#         # Convert matrices to sparse format for better performance
#         H = csc_matrix(H)
#         A = csc_matrix(A)

#         if with_slack:
#             H_slack = np.hstack((H.toarray(), np.zeros((H.shape[0], 1))))
#             H_slack = np.vstack((H_slack, np.hstack((np.zeros((1, H.shape[0])), self.weight_slack * np.ones((1, 1))))))

#             f_slack = np.hstack((f, 0))

#             A_slack = np.hstack((A.toarray(), np.zeros((A.shape[0], 1))))
#             lb_slack = np.hstack((lb, -np.inf))
#             ub_slack = np.hstack((ub, np.inf))

#             # Ensure all matrices are in sparse format for qpsolvers
#             H_slack = csc_matrix(H_slack)
#             A_slack = csc_matrix(A_slack)

#             u_slack = qp.solve_qp(P=H_slack, q=f_slack, G=A_slack, h=b, A=None, b=None, lb=lb_slack, ub=ub_slack, solver='clarabel', max_iter=10)

#             if u_slack is not None:
#                 u = u_slack[:self.udim]
#                 slack = u_slack[-1]
#                 feas = 1
#             else:
#                 u = None
#                 slack = None
#                 feas = -1
#         else:
#             u = qp.solve_qp(P=H, q=f, G=A, h=b, A=None, b=None, lb=lb, ub=ub, solver='clarabel', max_iter=10)

#             if u is not None:
#                 slack = None
#                 feas = 1
#             else:
#                 u = None
#                 slack = None
#                 feas = -1

#         return u, slack, h_prime, feas