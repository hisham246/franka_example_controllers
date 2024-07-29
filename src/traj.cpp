#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "traj");
  ros::NodeHandle nh;

  // Create a publisher to publish the target pose
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_example_controller/equilibrium_pose", 10);
  ros::Rate rate(10); // 10 Hz

  // Define the target pose
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "panda_link8";
  target_pose.header.stamp = ros::Time::now();

  // Set the target position (example values, change as needed)
  target_pose.pose.position.x = 5.5;
  target_pose.pose.position.y = 5.0;
  target_pose.pose.position.z = 5.5;

  // Set the orientation to a 90-degree rotation around the y-axis
  tf::Quaternion q;
  q.setRPY(0, M_PI_2, 0); // Roll, Pitch, Yaw (90 degrees = PI/2 radians)

  // Convert quaternion to geometry_msgs format
  target_pose.pose.orientation.x = q.x();
  target_pose.pose.orientation.y = q.y();
  target_pose.pose.orientation.z = q.z();
  target_pose.pose.orientation.w = q.w();

  // Publish the pose for a certain duration
  ros::Time start_time = ros::Time::now();
  ros::Duration duration(10.0); // Publish for 10 seconds

  while (ros::ok() && (ros::Time::now() - start_time) < duration) {
    target_pose.header.stamp = ros::Time::now(); // Update the timestamp
    pose_pub.publish(target_pose);
    rate.sleep();
  }

  return 0;
}