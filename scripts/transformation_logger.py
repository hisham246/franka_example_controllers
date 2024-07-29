#!/usr/bin/env python

import rospy
import json
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf

def transformation_matrix_from_pose(pose):
    position = pose.position
    orientation = pose.orientation
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    translation = [position.x, position.y, position.z]

    matrix = tf.quaternion_matrix(quaternion)
    matrix[0][3] = translation[0]
    matrix[1][3] = translation[1]
    matrix[2][3] = translation[2]

    print(matrix)

    return matrix.tolist()

class TransformationLogger:
    def __init__(self):
        rospy.init_node('transformation_logger', anonymous=True)

        self.transform_file = open('src/franka_ros/franka_example_controllers/results/end_effector_transform.json', 'w')
        self.transform_data = []

        rospy.Subscriber("/cartesian_impedance_example_controller/end_effector_transform", PoseStamped, self.transform_callback)

    def transform_callback(self, msg):
        time = rospy.Time.now().to_sec()
        matrix = transformation_matrix_from_pose(msg.pose)
        self.transform_data.append({
            'time': time,
            'transform_matrix': matrix
        })

    def run(self):
        rospy.spin()
        with open('src/franka_ros/franka_example_controllers/results/end_effector_transform.json', 'w') as f:
            json.dump(self.transform_data, f, indent=4)

if __name__ == '__main__':
    try:
        logger = TransformationLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass