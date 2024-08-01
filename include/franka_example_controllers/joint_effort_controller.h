#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace franka_example_controllers {

class JointEffortController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

private:
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::vector<Eigen::VectorXd> torques_;
  std::string torque_file_path_;
  size_t torque_index_;

  bool loadTorquesFromFile(const std::string& file_path);
};

} // namespace franka_example_controllers