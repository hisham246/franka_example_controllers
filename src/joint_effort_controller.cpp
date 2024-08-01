#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include <franka_example_controllers/joint_effort_controller.h>

namespace franka_example_controllers {

// bool CartesianImpedanceExampleController::init(hardware_interface::RobotHW* robot_hw,
//                                                ros::NodeHandle& node_handle) {

bool JointEffortController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) {
  // Get joint names from parameter server
  std::vector<std::string> joint_names;
  if (!nh.getParam("joints", joint_names) || joint_names.size() == 0) {
    ROS_ERROR("No joints given in namespace: '%s')", nh.getNamespace().c_str());
    return false;
  }

  // Get the CSV file path
  if (!nh.getParam("torque_file_path", torque_file_path_)) {
    ROS_ERROR("Parameter 'torque_file_path' not set in namespace: '%s'", nh.getNamespace().c_str());
    return false;
  }

  // Load joint handles
  for (const auto& joint_name : joint_names) {
    joint_handles_.push_back(hw->getHandle(joint_name));
  }

  // Load torques from the CSV file
  if (!loadTorquesFromFile(torque_file_path_)) {
    return false;
  }

  // Initialize the torque index
  torque_index_ = 0;

  return true;
}

void starting(const ros::Time& time) override {
  // Nothing to do here
}

void update(const ros::Time& time, const ros::Duration& period) override {
  if (torque_index_ < torques_.size()) {
    // Apply the torques to the joints
    for (size_t i = 0; i < joint_handles_.size(); ++i) {
      joint_handles_[i].setCommand(torques_[torque_index_][i]);
    }
    torque_index_++;
  } else {
    ROS_WARN_ONCE("JointEffortController: All torques from the file have been applied.");
  }
}

void stopping(const ros::Time& time) override {
  // Set all torques to zero
  for (auto& joint_handle : joint_handles_) {
    joint_handle.setCommand(0.0);
  }
}

private:
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::vector<Eigen::VectorXd> torques_;
  std::string torque_file_path_;
  size_t torque_index_;

  bool loadTorquesFromFile(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      ROS_ERROR_STREAM("JointEffortController: Could not open torque file: " << file_path);
      return false;
    }

    std::string line;
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string value;
      Eigen::VectorXd torque_row(7);  // Assuming 7 joints
      int i = 0;
      while (std::getline(ss, value, ',') && i < 7) {
        torque_row[i] = std::stod(value);
        ++i;
      }
      torques_.push_back(torque_row);
    }

    file.close();
    return true;
  }
};
 // namespace my_robot_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointEffortController, controller_interface::ControllerBase)
