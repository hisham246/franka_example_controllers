#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np

class PandaDynamicsModel:
    def __init__(self):
        self.coriolis = np.zeros(7)
        self.mass_matrix = np.zeros((7, 7))
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

if __name__ == '__main__':
    rospy.init_node('panda_dynamics_model', anonymous=True)
    panda_dynamics_model = PandaDynamicsModel()

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        rospy.loginfo("Mass Matrix: %s", panda_dynamics_model.inertia_matrix)
        rospy.loginfo("Coriolis: %s", panda_dynamics_model.coriolis)
        rospy.loginfo("Gravity Vector: %s", panda_dynamics_model.gravity_vector)

        rate.sleep()