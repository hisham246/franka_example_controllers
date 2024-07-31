// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_impedance_example_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka_example_controllers/pseudo_inversion.h>
#include <std_msgs/Float64MultiArray.h>
// #include <qpOASES.hpp>

namespace franka_example_controllers {

bool CartesianImpedanceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 20, &CartesianImpedanceExampleController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceExampleController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianImpedanceExampleController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  // Added
  transform_pub_ = node_handle.advertise<geometry_msgs::PoseStamped>("end_effector_transform", 10);

  desired_cartesian_state_pub_ = node_handle.advertise<geometry_msgs::PoseStamped>("desired_cartesian_state", 10);
  actual_cartesian_state_pub_ = node_handle.advertise<geometry_msgs::PoseStamped>("actual_cartesian_state", 10);
  desired_stiffness_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("desired_stiffness_state", 10);
  actual_stiffness_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("actual_stiffness_state", 10);

  // Dynamic model parameters nodes
  inertia_matrix_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("inertia_matrix", 10);
  coriolis_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("coriolis", 10);
  gravity_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("gravity", 10);

  tau_d_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("tau_d", 10);

  // sub_optimized_input_ = node_handle.subscribe(
  //     "control_input", 10, &CartesianImpedanceExampleController::optimizedInputCallback, this,
  //     ros::TransportHints().reliable().tcpNoDelay());

  jacobian_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("jacobian", 10);
  error_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("error", 10);

  return true;
}

void CartesianImpedanceExampleController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
}

void CartesianImpedanceExampleController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) { 

  // solveQPExample();

  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // Added: Mass matrix
  std::array<double, 49> mass_array = model_handle_->getMass();

  // Added: Gravity vector
  std::array<double, 7> gravity_array = model_handle_->getGravity();

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  // Added: Mass matrix
  Eigen::Map<Eigen::Matrix<double, 7, 7>> inertia_matrix(mass_array.data());

  // Added: Gravity vector
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());



  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.rotation() * error.tail(3);


  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

  publishDesiredTorques(tau_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

  // Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX())); // 90 degrees
  // orientation_d_target_ = rotation * orientation_d_target_;

  // Added
  // Publish the end effector transformation
  geometry_msgs::PoseStamped transform_msg;
  transform_msg.header.stamp = ros::Time::now();
  transform_msg.header.frame_id = "panda_link7";

  transform_msg.pose.position.x = position.x();
  transform_msg.pose.position.y = position.y();
  transform_msg.pose.position.z = position.z();

  transform_msg.pose.orientation.x = orientation.x();
  transform_msg.pose.orientation.y = orientation.y();
  transform_msg.pose.orientation.z = orientation.z();
  transform_msg.pose.orientation.w = orientation.w();

  transform_pub_.publish(transform_msg);


  // Publish the desired state
  CartesianState desired_cartesian_state;
  desired_cartesian_state.position = position_d_;
  desired_cartesian_state.orientation = orientation_d_;
  publishCartesianState(desired_cartesian_state, desired_cartesian_state_pub_);

  StiffnessState desired_stiffness_state;
  desired_stiffness_state.translational_stiffness = cartesian_stiffness_target_.diagonal().head(3);
  desired_stiffness_state.rotational_stiffness = cartesian_stiffness_target_.diagonal().tail(3);
  publishStiffnessState(desired_stiffness_state, desired_stiffness_pub_);

  // Publish the actual state
  CartesianState actual_cartesian_state;
  actual_cartesian_state.position = position;
  actual_cartesian_state.orientation = orientation;
  publishCartesianState(actual_cartesian_state, actual_cartesian_state_pub_);


  StiffnessState actual_stiffness_state;
  actual_stiffness_state.translational_stiffness = cartesian_stiffness_.diagonal().head(3);
  actual_stiffness_state.rotational_stiffness = cartesian_stiffness_.diagonal().tail(3); 
  publishStiffnessState(actual_stiffness_state, actual_stiffness_pub_);

  // Publish Dynamic model parameters
  publishInertia(inertia_matrix);
  publishCoriolis(coriolis);
  publishGravity(gravity);

  publishJacobian(jacobian);
  publishError(error);

}

Eigen::Matrix<double, 7, 1> CartesianImpedanceExampleController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianImpedanceExampleController::complianceParamCallback(
    franka_example_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}

void CartesianImpedanceExampleController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

// Added
void CartesianImpedanceExampleController::publishCartesianState(const CartesianState& state, const ros::Publisher& pub) {
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.pose.position.x = state.position.x();
  msg.pose.position.y = state.position.y();
  msg.pose.position.z = state.position.z();
  msg.pose.orientation.x = state.orientation.x();
  msg.pose.orientation.y = state.orientation.y();
  msg.pose.orientation.z = state.orientation.z();
  msg.pose.orientation.w = state.orientation.w();
  pub.publish(msg);

  // ROS_INFO_STREAM("Published cartesian state: " << msg);
}

void CartesianImpedanceExampleController::publishStiffnessState(const StiffnessState& state, const ros::Publisher& pub) {
  std_msgs::Float64MultiArray msg;
  msg.data.insert(msg.data.end(), state.translational_stiffness.data(), state.translational_stiffness.data() + state.translational_stiffness.size());
  msg.data.insert(msg.data.end(), state.rotational_stiffness.data(), state.rotational_stiffness.data() + state.rotational_stiffness.size());
  pub.publish(msg);
  // ROS_INFO_STREAM("Published stiffness state: " << msg);

}

// Publish dynamic model parameters

void CartesianImpedanceExampleController::publishInertia(const Eigen::Matrix<double, 7, 7>& inertia_matrix) {
    std_msgs::Float64MultiArray msg;
    for (int i = 0; i < inertia_matrix.size(); ++i) {
        msg.data.push_back(inertia_matrix.data()[i]);
    }
    inertia_matrix_pub_.publish(msg);
}

void CartesianImpedanceExampleController::publishCoriolis(const Eigen::Matrix<double, 7, 1>& coriolis) {
    std_msgs::Float64MultiArray msg;
    for (int i = 0; i < coriolis.size(); ++i) {
        msg.data.push_back(coriolis(i));
    }
    coriolis_pub_.publish(msg);
}

void CartesianImpedanceExampleController::publishGravity(const Eigen::Matrix<double, 7, 1>& gravity) {
    std_msgs::Float64MultiArray msg;
    for (int i = 0; i < gravity.size(); ++i) {
        msg.data.push_back(gravity(i));
    }
    gravity_pub_.publish(msg);
}

void CartesianImpedanceExampleController::publishDesiredTorques(const Eigen::Matrix<double, 7, 1>& tau_d) {
    std_msgs::Float64MultiArray msg;
    for (int i = 0; i < tau_d.size(); ++i) {
        msg.data.push_back(tau_d(i));
    }
    tau_d_pub_.publish(msg);
}

// Update control inputs
// void CartesianImpedanceExampleController::optimizedInputCallback(const std_msgs::Float64MultiArrayConstPtr& msg) {
//   if (msg->data.size() == 7) {
//     for (size_t i = 0; i < 7; ++i) {
//       joint_handles_[i].setCommand(msg->data[i]);
//     }
//   } else {
//     ROS_ERROR("CartesianImpedanceExampleController: Received optimized input of incorrect size");
//   }
// }

void CartesianImpedanceExampleController::publishJacobian(const Eigen::Matrix<double, 6, 7>& jacobian) {
    std_msgs::Float64MultiArray msg;
    for (int i = 0; i < jacobian.size(); ++i) {
        msg.data.push_back(jacobian.data()[i]);
    }
    jacobian_pub_.publish(msg);
}

void CartesianImpedanceExampleController::publishError(const Eigen::Matrix<double, 6, 1>& error) {
    std_msgs::Float64MultiArray msg;
    for (int i = 0; i < error.size(); ++i) {
        msg.data.push_back(error(i));
    }
    error_pub_.publish(msg);
}


// void CartesianImpedanceExampleController::solveQPExample() {
//   using namespace qpOASES;

//   /* Setup data of first QP. */
//   real_t H[2 * 2] = {1.0, 0.0, 0.0, 0.5};
//   real_t g[2] = {1.5, 1.0};
//   real_t A[1 * 2] = {1.0, 1.0};
//   real_t lb[2] = {0.5, -2.0};
//   real_t ub[2] = {5.0, 2.0};
//   real_t lbA[1] = {-1.0};
//   real_t ubA[1] = {2.0};

//   /* Setting up QProblem object. */
//   QProblem example(2, 1);

//   Options options;
//   example.setOptions(options);

//   /* Solve first QP. */
//   int_t nWSR = 10;
//   example.init(H, g, A, lb, ub, lbA, ubA, nWSR);

//   /* Get and print solution of first QP. */
//   real_t xOpt[2];
//   real_t yOpt[2 + 1];
//   example.getPrimalSolution(xOpt);
//   example.getDualSolution(yOpt);

//   ROS_INFO_STREAM("QP Solution: xOpt[0] = " << xOpt[0] << ", xOpt[1] = " << xOpt[1]);
// }


}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceExampleController,
                       controller_interface::ControllerBase)