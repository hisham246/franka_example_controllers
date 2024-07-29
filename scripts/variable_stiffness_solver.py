import rospy
import numpy as np
import casadi as ca
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class PandaJointStates:
    def __init__(self):
        self.positions = np.zeros(7)
        self.velocities = np.zeros(7)

        self.joint_states_sub = rospy.Subscriber('franka_state_controller/joint_states', JointState, self.joint_states_callback)

    def joint_states_callback(self, msg):
        self.positions = np.array(msg.position)
        self.velocities = np.array(msg.velocity)

class OptimizedTorques:
    def __init__(self):
        self.tau_star = np.zeros(7)
        self.tau_star_sub = rospy.Subscriber('tau_star', Float64MultiArray, self.tau_star_callback)

    def tau_star_callback(self, msg):
        self.tau_star = np.array(msg.data).reshape(7,1)

class Jacobian:
    def __init__(self):
        self.jacobian = np.zeros((6,7))
        self.jacobian_sub = rospy.Subscriber('cartesian_impedance_example_controller/jacobian', Float64MultiArray, self.jacobian_callback)
    
    def jacobian_callback(self, msg):
        self.jacobian = np.array(msg.data).reshape(6,7)

class Error:
    def __init__(self):
        self.error = np.zeros(6)
        self.error_sub = rospy.Subscriber('cartesian_impedance_example_controller/error', Float64MultiArray, self.error_callback)

    def error_callback(self, msg):
        self.error = np.array(msg.data).reshape(6,1)


def solve_kp(A, x_tilde, B):
    # Define the optimization variables (diagonal elements of Kp)
    k = ca.MX.sym('k', 6)

    # Define Kp and sqrt(Kp)
    Kp_diag = ca.diag(k)
    sqrt_Kp_diag = ca.diag(ca.sqrt(k))

    # Compute the residual
    residual = A - (Kp_diag @ x_tilde + sqrt_Kp_diag @ B)
    objective = ca.sum1(ca.sum2(residual**2))

    # Define the optimization problem
    nlp = {'x': k, 'f': objective}

    # Create an NLP solver
    opts = {"ipopt.print_level": 0, "print_time": 0}
    solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

    # Solve the problem
    kp_initial_guess = np.ones(6)  # Initial guess
    lbx = np.full(6, 1e-6)  # Lower bounds to ensure positive definiteness
    ubx = np.full(6, 100)  # No upper bounds

    solution = solver(x0=kp_initial_guess, lbx=lbx, ubx=ubx)

    # Extract the optimized values of k
    kp_optimized = solution['x'].full().flatten()
    return np.diag(kp_optimized)

if __name__ == '__main__':
    rospy.init_node('variable_stiffness_solver', anonymous=True)

    tau_star_ = OptimizedTorques()
    jacobian_ =  Jacobian()
    panda_joint_states_ = PandaJointStates()
    error_ = Error()


    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        tau_star = tau_star_.tau_star
        jacobian = jacobian_.jacobian
        dq = panda_joint_states_.velocities
        x_tilde = error_.error

        # Calculate A and B
        A = np.linalg.pinv(jacobian.T) @ tau_star 
        B = -2 * (jacobian @ dq) 

        Kp_star = solve_kp(A, x_tilde, B)
        rospy.loginfo("Stiffness matrix: %s", Kp_star)

        rate.sleep()
