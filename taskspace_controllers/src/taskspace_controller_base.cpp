// Copyright 2024 Alex Arbogast
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "taskspace_controllers/taskspace_controller_base.hpp"
#include "taskspace_controllers/utility.hpp"

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "urdf/model.h"
#include "joint_limits/joint_limits_urdf.hpp"

namespace taskspace_controllers
{

controller_interface::InterfaceConfiguration
TaskspaceControllerBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(n_joints_ * params_.command_interfaces.size());
  for (const auto& type : params_.command_interfaces)
  {
    for (const auto& joint : joint_names_)
    {
      cfg.names.push_back(joint + std::string("/").append(type));
    }
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
TaskspaceControllerBase::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  cfg.names.reserve(n_joints_ * params_.state_interfaces.size());
  for (const auto& type : params_.state_interfaces)
  {
    for (const auto& joint : joint_names_)
    {
      cfg.names.push_back(joint + std::string("/").append(type));
    }
  }
  return cfg;
}

controller_interface::CallbackReturn TaskspaceControllerBase::on_init()
{
  try
  {
    param_listener_ =
        std::make_shared<taskspace_controller_base::ParamListener>(get_node());
  }
  catch (const std::exception& e)
  {
    fprintf(stderr,
            "Exception thrown during controller's init with message: %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  auto_declare<std::string>("robot_description", "");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TaskspaceControllerBase::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  auto logger = get_node()->get_logger();
  params_ = param_listener_->get_params();

  has_position_command_interface_ = ctrl::contains_interface_type(
      params_.command_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_command_interface_ = ctrl::contains_interface_type(
      params_.command_interfaces, hardware_interface::HW_IF_VELOCITY);

  has_position_state_interface_ = ctrl::contains_interface_type(
      params_.state_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_state_interface_ = ctrl::contains_interface_type(
      params_.state_interfaces, hardware_interface::HW_IF_VELOCITY);

  // Find interface based on type instead of assuming order
  if (!has_position_state_interface_)
  {
    RCLCPP_FATAL(logger,
                 "TaskspaceControllerBase requires a position state interface");
    return controller_interface::CallbackReturn::ERROR;
  }

  position_state_interface_index_ =
      std::distance(params_.state_interfaces.begin(),
                    std::find(params_.state_interfaces.begin(),
                              params_.state_interfaces.end(),
                              hardware_interface::HW_IF_POSITION));

  if (has_velocity_state_interface_)
  {
    velocity_state_interface_index_ =
        std::distance(params_.state_interfaces.begin(),
                      std::find(params_.state_interfaces.begin(),
                                params_.state_interfaces.end(),
                                hardware_interface::HW_IF_VELOCITY));
  }

  std::string urdf_xml;
#ifdef TASKSPACE_CONTROLLERS_JAZZY
  urdf_xml = this->get_robot_description();
#else
  urdf_xml = get_node()->get_parameter("robot_description").as_string();
#endif

  if (urdf_xml.empty())
  {
    RCLCPP_ERROR(logger, "robot_description is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  base_link_ = params_.base_link;
  eef_link_ = params_.eef_link;

  // parse URDF -> KDL
  urdf::Model urdf_model;
  KDL::Tree kdl_tree;
  if (!urdf_model.initString(urdf_xml))
  {
    RCLCPP_ERROR(logger,
                 "Failed to parse URDF from 'robot_description' parameter.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree))
  {
    RCLCPP_FATAL(logger, "Failed to convert URDF to KDL tree.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!kdl_tree.getChain(base_link_, eef_link_, robot_chain_))
  {
    RCLCPP_FATAL(logger, "Failed to build KDL chain from '%s' to '%s'.",
                 base_link_.c_str(), eef_link_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_names_ = ctrl::joints_along_chain(robot_chain_);
  n_joints_ = joint_names_.size();

  // allocate dynamic memory
  last_reference_.resize(n_joints_);
  last_commanded_ = last_reference_;
  joint_state_ = last_reference_;

  // Initialize the joint limits from the urdf
  joint_limits_.resize(n_joints_);
  for (size_t i = 0; i < n_joints_; ++i)
  {
    const auto& jn = joint_names_[i];
    auto j = urdf_model.getJoint(jn);
    if (!j)
    {
      RCLCPP_ERROR(logger, "Joint '%s' not found in URDF.", jn.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    joint_limits::getJointLimits(j, joint_limits_[i]);
  }

  robot_fk_solver_ =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(robot_chain_);

  // Create service for query_pose
  query_pose_service_ = get_node()->create_service<QueryPose>(
      get_node()->get_name() + std::string("/query_pose"),
      std::bind(&TaskspaceControllerBase::queryPoseServiceCb, this,
                std::placeholders::_1, std::placeholders::_2));

  state_pub_ = get_node()
                   ->create_publisher<
                       taskspace_control_msgs::msg::TaskspaceControlDiagnostic>(
                       get_node()->get_name() + std::string("/joint_state_"
                                                            "diagnostic"),
                       10);
  rt_state_pub_ = std::make_unique<realtime_tools::RealtimePublisher<
      taskspace_control_msgs::msg::TaskspaceControlDiagnostic>>(state_pub_);
  diagnostic_msg_.command.name = joint_names_;
  diagnostic_msg_.command.position.resize(n_joints_);
  diagnostic_msg_.command.velocity.resize(n_joints_);
  diagnostic_msg_.state.name = joint_names_;
  diagnostic_msg_.state.position.resize(n_joints_);
  diagnostic_msg_.state.velocity.resize(n_joints_);
  diagnostic_msg_.state_error.name = joint_names_;
  diagnostic_msg_.state_error.position.resize(n_joints_);
  diagnostic_msg_.state_error.velocity.resize(n_joints_);

  RCLCPP_INFO(logger, "TaskspaceControllerBase configured for %u joints.",
              n_joints_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TaskspaceControllerBase::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  RCLCPP_INFO(logger, "Activated TaskspaceControllerBase");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TaskspaceControllerBase::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // derived classes will compute command vector and call write_command()
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn TaskspaceControllerBase::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  stop_motion();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TaskspaceControllerBase::on_shutdown(
    const rclcpp_lifecycle::State& previous_state)
{
  stop_motion();
  return controller_interface::CallbackReturn::SUCCESS;
}

void TaskspaceControllerBase::read_state_from_hardware(KDL::JntArrayVel& state)
{
  bool nan_position = false;
  for (size_t joint_ind = 0; joint_ind < n_joints_; ++joint_ind)
  {
    state.q(joint_ind) =
        state_interfaces_[position_state_interface_index_ * n_joints_ +
                          joint_ind]
            .get_value();
    nan_position |= std::isnan(state.q(joint_ind));
  }

  if (nan_position)
  {
    state.q = last_commanded_.q;
  }

  bool nan_velocity = false;
  if (has_velocity_state_interface_)
  {
    for (size_t joint_ind = 0; joint_ind < n_joints_; ++joint_ind)
    {
      state.qdot(joint_ind) =
          state_interfaces_[velocity_state_interface_index_ * n_joints_ +
                            joint_ind]
              .get_value();
      nan_velocity |= std::isnan(state.qdot(joint_ind));
    }
  }
  else
  {
    for (size_t joint_ind = 0; joint_ind < n_joints_; ++joint_ind)
    {
      state.qdot(joint_ind) = 0.0;
    }
  }

  if (nan_velocity)
  {
    state.qdot = last_commanded_.qdot;
  }
}

void TaskspaceControllerBase::write_command(const KDL::JntArrayVel& cmd)
{
  size_t pos_ind = 0;
  size_t vel_ind = (has_position_command_interface_) ?
                       pos_ind + has_velocity_command_interface_ :
                       pos_ind;
  for (size_t joint_ind = 0; joint_ind < n_joints_; ++joint_ind)
  {
    if (has_position_command_interface_)
    {
      command_interfaces_[pos_ind * n_joints_ + joint_ind].set_value(
          cmd.q(joint_ind));
    }
    if (has_velocity_command_interface_)
    {
      command_interfaces_[vel_ind * n_joints_ + joint_ind].set_value(
          cmd.qdot(joint_ind));
    }
  }
  last_commanded_ = cmd;
}

void TaskspaceControllerBase::stop_motion()
{
  // Stop motion for velocity control
  if (has_velocity_command_interface_)
  {
    size_t pos_ind = 0;
    size_t vel_ind = (has_position_command_interface_) ?
                         pos_ind + has_velocity_command_interface_ :
                         pos_ind;

    for (size_t joint_ind = 0; joint_ind < n_joints_; ++joint_ind)
    {
      command_interfaces_[vel_ind * n_joints_ + joint_ind].set_value(0.0);
    }
  }
}

bool TaskspaceControllerBase::check_manipulability(const KDL::Jacobian& jac)
{
  const double w = ctrl::compute_manipulability(jac);
  if (w < params_.manipulability_threshold)
  {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                         500 /*ms*/,
                         "Manipulability (%.6f) below threshold (%.6f) — "
                         "zeroing output.",
                         w, params_.manipulability_threshold);
    stop_motion();
    return false;
  }
  return true;
}

bool TaskspaceControllerBase::queryPoseServiceCb(
    const std::shared_ptr<QueryPose::Request> /*req*/,
    std::shared_ptr<QueryPose::Response> resp)
{
  read_state_from_hardware(joint_state_);

  KDL::Frame pose;
  if (!robot_fk_solver_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "FK solver not initialized.");
    return false;
  }
  int fk_res = robot_fk_solver_->JntToCart(joint_state_.q, pose);
  if (fk_res < 0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "FK solver failed.");
    return false;
  }

  resp->pose.position.x = pose.p.x();
  resp->pose.position.y = pose.p.y();
  resp->pose.position.z = pose.p.z();

  pose.M.GetQuaternion(resp->pose.orientation.x, resp->pose.orientation.y,
                       resp->pose.orientation.z, resp->pose.orientation.w);

  return true;
}

void TaskspaceControllerBase::publish_state_diagnostic(
    const rclcpp::Time& time, const KDL::JntArrayVel& joint_cmd,
    const KDL::JntArrayVel& joint_state, const KDL::JntArrayVel& state_error,
    const ctrl::Pose& sp_pose, const ctrl::Pose& pose,
    const ctrl::Vector3D& trans_error, const ctrl::Vector3D& orient_error)
{
  if (!rt_state_pub_)
  {
    return;
  }

  // Match update time instead of node time
  diagnostic_msg_.command.header.stamp = time;
  diagnostic_msg_.state.header.stamp = time;
  diagnostic_msg_.state_error.header.stamp = time;

  for (size_t joint_ind = 0; joint_ind < n_joints_; ++joint_ind)
  {
    diagnostic_msg_.command.position[joint_ind] = joint_cmd.q(joint_ind);
    diagnostic_msg_.command.velocity[joint_ind] = joint_cmd.qdot(joint_ind);
    diagnostic_msg_.state.position[joint_ind] = joint_state.q(joint_ind);
    diagnostic_msg_.state.velocity[joint_ind] = joint_state.qdot(joint_ind);
    diagnostic_msg_.state_error.position[joint_ind] = state_error.q(joint_ind);
    diagnostic_msg_.state_error.velocity[joint_ind] =
        state_error.qdot(joint_ind);
  }

  diagnostic_msg_.setpoint = ctrl::transformEigenToROS(sp_pose);
  diagnostic_msg_.pose = ctrl::transformEigenToROS(pose);
  diagnostic_msg_.pose_error.position.x = trans_error.x();
  diagnostic_msg_.pose_error.position.y = trans_error.y();
  diagnostic_msg_.pose_error.position.z = trans_error.z();
  diagnostic_msg_.pose_error.orientation.x = 0.0;
  diagnostic_msg_.pose_error.orientation.y = 0.0;
  diagnostic_msg_.pose_error.orientation.z = 0.0;
  diagnostic_msg_.pose_error.orientation.w = 1.0;

  if (orient_error.norm() > 0.0)
  {
    const ctrl::AngleAxis aa(orient_error.norm(), orient_error.normalized());
    const ctrl::Quaternion q_error(aa);
    diagnostic_msg_.pose_error.orientation.x = q_error.x();
    diagnostic_msg_.pose_error.orientation.y = q_error.y();
    diagnostic_msg_.pose_error.orientation.z = q_error.z();
    diagnostic_msg_.pose_error.orientation.w = q_error.w();
  }

#ifdef TASKSPACE_CONTROLLERS_JAZZY
  rt_state_pub_->try_publish(diagnostic_msg_);
#else
  rt_state_pub_->lock();
  rt_state_pub_->msg_ = diagnostic_msg_;
  rt_state_pub_->unlockAndPublish();
#endif
}

}  // namespace taskspace_controllers
