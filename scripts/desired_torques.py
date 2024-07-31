#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import pandas as pd
import numpy as np

class DesiredTorques:
    def __init__(self):
        self.tau_d = np.zeros(7)
        self.tau_d_sub = rospy.Subscriber('cartesian_impedance_example_controller/tau_d', Float64MultiArray, self.tau_d_callback)

    def tau_d_callback(self, msg):
        self.tau_d = np.array(msg.data)

def shutdown_callback(event=None):
    tau_d_df = pd.DataFrame(tau_d_list, columns=[f'Joint {i+1}' for i in range(7)])
    tau_d_df.to_csv('tau_nom_1khz.csv', index=False)

if __name__ == '__main__':
    rospy.init_node('desired_torques', anonymous=True)

    torques = DesiredTorques()

    rate = rospy.Rate(1000)
    
    rospy.Timer(rospy.Duration(10), shutdown_callback, oneshot=True)

    tau_d_list = []

    while not rospy.is_shutdown():

        tau_d_list.append(torques)
    
        rate.sleep()


    
