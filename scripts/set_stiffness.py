#!/usr/bin/env python

import rospy
from dynamic_reconfigure.client import Client
import time

def set_stiffness():
    rospy.init_node('set_stiffness', anonymous=True)

    # Create a dynamic reconfigure client
    client = Client("/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node")

    # Define a list of stiffness values to send
    stiffness_values = [
        {'translational_stiffness': 300.0, 'rotational_stiffness': 50.0, 'nullspace_stiffness': 20.0},
        {'translational_stiffness': 400.0, 'rotational_stiffness': 100.0, 'nullspace_stiffness': 40.0},
        {'translational_stiffness': 500.0, 'rotational_stiffness': 150.0, 'nullspace_stiffness': 60.0},
        {'translational_stiffness': 600.0, 'rotational_stiffness': 200.0, 'nullspace_stiffness': 80.0},
    ]

    for stiffness in stiffness_values:
        rospy.loginfo(f"Setting stiffness: {stiffness}")
        client.update_configuration(stiffness)
        time.sleep(5)  # Wait for 5 seconds before setting the next value

if __name__ == "__main__":
    try:
        set_stiffness()
    except rospy.ROSInterruptException:
        pass
