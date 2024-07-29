#!/usr/bin/env python

import rospy
import math
import pandas as pd
from dynamic_reconfigure.client import Client
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import tf.transformations as tf

def euler_from_quaternion(quat):
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    roll, pitch, yaw = tf.euler_from_quaternion([x, y, z, w])
    return roll, pitch, yaw

class StateLoggerAndController:
    def __init__(self):
        rospy.init_node('variable_stiffness_controller', anonymous=True)

        # State data lists
        self.desired_cartesian_state_data = []
        self.actual_cartesian_state_data = []
        self.desired_stiffness_state_data = []
        self.actual_stiffness_state_data = []

        # Subscribers
        rospy.Subscriber("cartesian_impedance_example_controller/desired_cartesian_state", PoseStamped, self.desired_cartesian_state_callback)
        rospy.Subscriber("cartesian_impedance_example_controller/actual_cartesian_state", PoseStamped, self.actual_cartesian_state_callback)
        rospy.Subscriber("cartesian_impedance_example_controller/desired_stiffness_state", Float64MultiArray, self.desired_stiffness_state_callback)
        rospy.Subscriber("cartesian_impedance_example_controller/actual_stiffness_state", Float64MultiArray, self.actual_stiffness_state_callback)

        # Dynamic reconfigure client
        self.client = Client('/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node')
        self.start_time = rospy.Time.now().to_sec()
        self.rate = rospy.Rate(10)  # 10 Hz

        rospy.loginfo("StateLogger and VariableStiffnessController node initialized")

    # Callback methods
    def desired_cartesian_state_callback(self, msg):
        self.log_cartesian_state('desired', msg)

    def actual_cartesian_state_callback(self, msg):
        self.log_cartesian_state('actual', msg)

    def desired_stiffness_state_callback(self, msg):
        self.log_stiffness_state('desired', msg)

    def actual_stiffness_state_callback(self, msg):
        self.log_stiffness_state('actual', msg)

    # Logging methods
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

    # Stiffness update method
    def update_stiffness(self):
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - self.start_time

        translational_stiffness = 200 + 100 * math.sin(elapsed_time)
        rotational_stiffness = 10 + 5 * math.sin(elapsed_time)

        rospy.loginfo("Translational Stiffness: %f, Rotational Stiffness: %f", 
                      translational_stiffness, rotational_stiffness)

        self.client.update_configuration({
            'translational_stiffness': translational_stiffness,
            'rotational_stiffness': rotational_stiffness,
        })

        # Log desired stiffness state
        time = rospy.Time.now().to_sec()
        # self.desired_stiffness_state_data.append([time, translational_stiffness, translational_stiffness, translational_stiffness,
        #                                           rotational_stiffness, rotational_stiffness, rotational_stiffness])

    def run(self):
        while not rospy.is_shutdown():
            self.update_stiffness()
            self.rate.sleep()
        self.save_data()

if __name__ == '__main__':
    try:
        controller = StateLoggerAndController()
        controller.run()
    except rospy.ROSInterruptException:
        pass


# #!/usr/bin/env python

# import rospy
# import math
# from dynamic_reconfigure.client import Client

# class VariableStiffnessController:
#     def __init__(self):
#         rospy.init_node('variable_stiffness_controller', anonymous=True)

#         self.client = Client('/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node')
#         self.start_time = rospy.Time.now().to_sec()
#         self.rate = rospy.Rate(10)  # 10 Hz

#     def update_stiffness(self):
#         current_time = rospy.Time.now().to_sec()
#         elapsed_time = current_time - self.start_time

#         translational_stiffness = 200 + 100 * math.sin(elapsed_time)
#         rotational_stiffness = 10 + 5 * math.sin(elapsed_time)

#         rospy.loginfo("Translational Stiffness: %f, Rotational Stiffness: %f", 
#                       translational_stiffness, rotational_stiffness)

#         self.client.update_configuration({
#             'translational_stiffness': translational_stiffness,
#             'rotational_stiffness': rotational_stiffness,
#         })

#     def run(self):
#         while not rospy.is_shutdown():
#             self.update_stiffness()
#             self.rate.sleep()

# if __name__ == '__main__':
#     try:
#         controller = VariableStiffnessController()
#         controller.run()
#     except rospy.ROSInterruptException:
#         pass