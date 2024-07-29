#!/usr/bin/env python

import rospy
import pandas as pd
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import tf.transformations as tf

def euler_from_quaternion(quat):
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    roll, pitch, yaw = tf.euler_from_quaternion([x, y, z, w])
    return roll, pitch, yaw

class StateLogger:
    def __init__(self):
        rospy.init_node('state_logger', anonymous=True)

        self.desired_cartesian_state_data = []
        self.actual_cartesian_state_data = []
        self.desired_stiffness_state_data = []
        self.actual_stiffness_state_data = []

        rospy.Subscriber("cartesian_impedance_example_controller/desired_cartesian_state", PoseStamped, self.desired_cartesian_state_callback)
        rospy.Subscriber("cartesian_impedance_example_controller/actual_cartesian_state", PoseStamped, self.actual_cartesian_state_callback)
        rospy.Subscriber("cartesian_impedance_example_controller/desired_stiffness_state", Float64MultiArray, self.desired_stiffness_state_callback)
        rospy.Subscriber("cartesian_impedance_example_controller/actual_stiffness_state", Float64MultiArray, self.actual_stiffness_state_callback)

        rospy.loginfo("StateLogger node initialized and subscribers set up")

    def desired_cartesian_state_callback(self, msg):
        self.log_cartesian_state('desired', msg)

    def actual_cartesian_state_callback(self, msg):
        self.log_cartesian_state('actual', msg)

    def desired_stiffness_state_callback(self, msg):
        self.log_stiffness_state('desired', msg)

    def actual_stiffness_state_callback(self, msg):
        self.log_stiffness_state('actual', msg)

    def log_cartesian_state(self, state_type, msg):
        time = rospy.Time.now().to_sec()
        position = msg.pose.position
        roll, pitch, yaw = euler_from_quaternion(msg.pose.orientation)
        if state_type == 'desired':
            self.desired_cartesian_state_data.append([time, position.x, position.y, position.z, roll, pitch, yaw])
        else:
            self.actual_cartesian_state_data.append([time, position.x, position.y, position.z, roll, pitch, yaw])
        rospy.loginfo(f"Logged {state_type} cartesian state at time {time}")

    def log_stiffness_state(self, state_type, msg):
        time = rospy.Time.now().to_sec()
        data = msg.data
        if state_type == 'desired':
            self.desired_stiffness_state_data.append([time] + list(data))
        else:
            self.actual_stiffness_state_data.append([time] + list(data))
        rospy.loginfo(f"Logged {state_type} stiffness state at time {time}")

    def save_data(self):
        desired_cartesian_state_df = pd.DataFrame(self.desired_cartesian_state_data, columns=['time', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'])
        actual_cartesian_state_df = pd.DataFrame(self.actual_cartesian_state_data, columns=['time', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'])
        desired_stiffness_state_df = pd.DataFrame(self.desired_stiffness_state_data, columns=['time', 'tx', 'ty', 'tz', 'rx', 'ry', 'rz'])
        actual_stiffness_state_df = pd.DataFrame(self.actual_stiffness_state_data, columns=['time', 'tx', 'ty', 'tz', 'rx', 'ry', 'rz'])

        rospy.loginfo("Saving Desired Cartesian state data to CSV")
        desired_cartesian_state_df.to_csv('src/franka_ros/franka_example_controllers/results/desired_cartesian_state.csv', index=False)
        rospy.loginfo("Saving Actual Cartesian state data to CSV")
        actual_cartesian_state_df.to_csv('src/franka_ros/franka_example_controllers/results/actual_cartesian_state.csv', index=False)
        rospy.loginfo("Saving Desired Stiffness state data to CSV")
        desired_stiffness_state_df.to_csv('src/franka_ros/franka_example_controllers/results/desired_stiffness_state.csv', index=False)
        rospy.loginfo("Saving Actual Stiffness state data to CSV")
        actual_stiffness_state_df.to_csv('src/franka_ros/franka_example_controllers/results/actual_stiffness_state.csv', index=False)

        rospy.loginfo("Desired Cartesian State Data:")
        rospy.loginfo(desired_cartesian_state_df)
        rospy.loginfo("Actual Cartesian State Data:")
        rospy.loginfo(actual_cartesian_state_df)
        rospy.loginfo("Desired Stiffness State Data:")
        rospy.loginfo(desired_stiffness_state_df)
        rospy.loginfo("Actual Stiffness State Data:")
        rospy.loginfo(actual_stiffness_state_df)

    def run(self):
        rospy.on_shutdown(self.save_data)
        rospy.spin()

if __name__ == '__main__':
    try:
        logger = StateLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass
