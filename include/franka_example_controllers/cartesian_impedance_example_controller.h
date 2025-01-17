// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <fstream>
#include <sstream>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <eigen3/Eigen/Dense>

#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <std_msgs/Float64MultiArray.h>
// #include <qpOASES.hpp>

namespace franka_example_controllers {

class CartesianImpedanceExampleController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  std::vector<Eigen::VectorXd> tau_d_from_file_;
  bool readTorquesFromFile(const std::string& file_path);

  size_t update_counter_ = 0; // Add this line to your class definition

  void writeTorquesToFile(const Eigen::Matrix<double, 7, 1>& tau_d);

  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  std::mutex position_and_orientation_d_target_mutex_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void complianceParamCallback(franka_example_controllers::compliance_paramConfig& config,
                               uint32_t level);

  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  std::vector<Eigen::VectorXd> desired_torques_;

  // Optimized input subscriber
  // ros::Subscriber sub_optimized_input_;
  // void optimizedInputCallback(const std_msgs::Float64MultiArrayConstPtr& msg);

// Added
  ros::Publisher transform_pub_; // End-effector transformation matrix

  ros::Publisher desired_cartesian_state_pub_;
  ros::Publisher actual_cartesian_state_pub_;
  ros::Publisher desired_stiffness_pub_;
  ros::Publisher actual_stiffness_pub_;

  // Publish the dynamic model parameters
  ros::Publisher inertia_matrix_pub_;
  ros::Publisher coriolis_pub_;
  ros::Publisher gravity_pub_;

  // Publish the desired control input 
  ros::Publisher tau_d_pub_;

  ros::Publisher jacobian_pub_;
  ros::Publisher error_pub_;

  struct CartesianState {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
  };
  struct StiffnessState {
    Eigen::Vector3d translational_stiffness;
    Eigen::Vector3d rotational_stiffness;
  };
  void publishCartesianState(const CartesianState& state, const ros::Publisher& pub);
  void publishStiffnessState(const StiffnessState& state, const ros::Publisher& pub);
  void publishInertia(const Eigen::Matrix<double, 7, 7>& inertia_matrix);
  void publishCoriolis(const Eigen::Matrix<double, 7, 1>& coriolis);
  void publishGravity(const Eigen::Matrix<double, 7, 1>& gravity);

  void publishDesiredTorques(const Eigen::Matrix<double, 7, 1>& tau_d);

  void publishJacobian(const Eigen::Matrix<double, 6, 7>& jacobian);
  void publishError(const Eigen::Matrix<double, 6, 1>& error);

  bool readTorquesFromFile();

  // void solveQPExample();

};

}  // namespace franka_example_controllers