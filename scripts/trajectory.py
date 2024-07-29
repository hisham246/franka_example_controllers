#!/usr/bin/env python

import rospy
import tf.transformations as tf
from geometry_msgs.msg import PoseStamped
import math

def publish_trajectory():
    rospy.init_node('trajectory', anonymous=True)
    trajectory_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    start_time = rospy.Time.now().to_sec()
    duration = 10  # run for 10 seconds

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - start_time

        if elapsed_time > duration:
            break

        # Define a simple circular trajectory
        radius = 6.0
        frequency = 0.1
        x = 0.5 + radius * math.cos(2 * math.pi * frequency * elapsed_time)
        y = 0.0 + radius * math.sin(2 * math.pi * frequency * elapsed_time)
        z = 0.5

        # Define a simple rotation
        roll = 10.0 * frequency * elapsed_time
        pitch = 20.0 * frequency * elapsed_time
        # yaw = 2 * math.pi * frequency * elapsed_time
        yaw = 10.0 * frequency * elapsed_time
        quaternion = tf.quaternion_from_euler(roll, pitch, yaw)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "panda_link0"  # Update this frame id as per your setup
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        rospy.loginfo(f"Orientation: {quaternion}")

        trajectory_pub.publish(pose_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_trajectory()
    except rospy.ROSInterruptException:
        pass
