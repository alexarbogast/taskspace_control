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
#include <cmath>
#include <algorithm>
#include "taskspace_controllers/taskspace_controller_base.hpp"
#include "taskspace_controllers/utility.hpp"

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "lifecycle_msgs/msg/state.hpp"
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

  for (const auto& type : { hardware_interface::HW_IF_POSITION,
                            hardware_interface::HW_IF_VELOCITY })
  {
    if (!ctrl::contains_interface_type(params_.command_interfaces, type))
    {
      continue;
    }

    for (const auto& joint : joint_names_)
    {
      cfg.names.push_back(joint + "/" + type);
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

  for (const auto& type : { hardware_interface::HW_IF_POSITION,
                            hardware_interface::HW_IF_VELOCITY })
  {
    if (!ctrl::contains_interface_type(params_.state_interfaces, type))
    {
      continue;
    }

    for (const auto& joint : joint_names_)
    {
      cfg.names.push_back(joint + "/" + type);
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

  integration_mode_ = params_.joint_position_integration_mode == "open_loop" ?
                          JointPositionIntegrationMode::OPEN_LOOP :
                          JointPositionIntegrationMode::CLOSED_LOOP;

  // Find interface based on type instead of assuming order
  if (!has_position_state_interface_)
  {
    RCLCPP_FATAL(logger,
                 "TaskspaceControllerBase requires a position state interface");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (integration_mode_ == JointPositionIntegrationMode::OPEN_LOOP)
  {
    if (!has_position_command_interface_)
    {
      RCLCPP_FATAL(
          logger,
          "Open-loop joint-position integration requires a position command "
          "interface.");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (params_.open_loop_reference_error_limit <= 0.0 ||
        !std::isfinite(params_.open_loop_reference_error_limit))
    {
      RCLCPP_FATAL(
          logger,
          "open_loop_reference_error_limit must be finite and positive when "
          "joint_position_integration_mode is 'open_loop'.");
      return controller_interface::CallbackReturn::ERROR;
    }
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
  joint_command_.resize(n_joints_);
  joint_command_prev_.resize(n_joints_);
  joint_state_.resize(n_joints_);
  joint_limits_.resize(n_joints_);

  KDL::SetToZero(joint_command_);
  KDL::SetToZero(joint_command_prev_);
  KDL::SetToZero(joint_state_);

  // Initialize the joint limits from the urdf
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

  RCLCPP_INFO(get_node()->get_logger(), "Diagnostics enabled: %s",
              params_.enable_diagnostics ? "true" : "false");
  if (params_.enable_diagnostics)
  {
    diagnostic_pub_ =
        get_node()->create_publisher<taskspace_control_msgs::msg::Diagnostic>(
            "~/diagnostics", rclcpp::SystemDefaultsQoS());
    rt_diagnostic_pub_ = std::make_unique<realtime_tools::RealtimePublisher<
        taskspace_control_msgs::msg::Diagnostic>>(diagnostic_pub_);

    auto init_joint_state = [&](auto& js) {
      js.name = joint_names_;
      js.position.resize(n_joints_);
      js.velocity.resize(n_joints_);
    };

    init_joint_state(rt_diagnostic_pub_->msg_.command);
    init_joint_state(rt_diagnostic_pub_->msg_.state);
    init_joint_state(rt_diagnostic_pub_->msg_.state_error);
  }

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

  if (!read_state_from_hardware(joint_state_))
  {
    write_zero_velocity_command();
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_command_prev_.q = joint_state_.q;
  joint_command_prev_.qdot.data.setZero();

  RCLCPP_INFO(logger, "Activated TaskspaceControllerBase");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TaskspaceControllerBase::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // derived classes will compute command vector and call write_joint_command()
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
  if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    stop_motion();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool TaskspaceControllerBase::read_state_from_hardware(KDL::JntArrayVel& state)
{
  bool invalid_position = false;
  for (size_t joint_ind = 0; joint_ind < n_joints_; ++joint_ind)
  {
    state.q(joint_ind) = state_interfaces_[joint_ind].get_value();
    invalid_position |= !std::isfinite(state.q(joint_ind));
  }
  if (invalid_position)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Position state interface returned an invalid value.");
  }

  bool invalid_velocity = false;
  if (has_velocity_state_interface_)
  {
    const auto velocity_offset = has_position_state_interface_ ? n_joints_ : 0;

    for (size_t joint_ind = 0; joint_ind < n_joints_; ++joint_ind)
    {
      state.qdot(joint_ind) =
          state_interfaces_[velocity_offset + joint_ind].get_value();
      invalid_velocity |= !std::isfinite(state.qdot(joint_ind));
    }
    if (invalid_velocity)
    {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Velocity state interface returned an invalid value.");
    }
  }
  else
  {
    state.qdot.data.setZero();
  }

  return !invalid_position && !invalid_velocity;
}

const KDL::JntArrayVel& TaskspaceControllerBase::update_joint_command(
    const ctrl::VectorND& joint_velocity_command,
    const rclcpp::Duration& period)
{
  if (period.nanoseconds() <= 0)
  {
    joint_command_prev_.qdot.data.setZero();
    return joint_command_prev_;
  }
  const double dt = period.seconds();

  const ctrl::VectorND& integration_origin =
      integration_mode_ == JointPositionIntegrationMode::OPEN_LOOP ?
          joint_command_prev_.q.data :
          joint_state_.q.data;

  ctrl::integrate_joint_velocity(integration_origin, joint_velocity_command,
                                 joint_limits_, dt, joint_command_);

  if (integration_mode_ == JointPositionIntegrationMode::OPEN_LOOP)
  {
    const double error_limit = params_.open_loop_reference_error_limit;
    double scale = 1.0;
    for (size_t i = 0; i < n_joints_; ++i)
    {
      const double position_increment =
          joint_command_.q(i) - joint_command_prev_.q(i);
      const double reference_error =
          joint_command_prev_.q(i) - joint_state_.q(i);

      if (position_increment > 0.0)
      {
        scale = std::min(scale, std::clamp((error_limit - reference_error) /
                                               position_increment,
                                           0.0, 1.0));
      }
      else if (position_increment < 0.0)
      {
        scale = std::min(scale, std::clamp((-error_limit - reference_error) /
                                               position_increment,
                                           0.0, 1.0));
      }
    }

    if (scale < 1.0)
    {
      for (size_t i = 0; i < n_joints_; ++i)
      {
        const double position_increment =
            joint_command_.q(i) - joint_command_prev_.q(i);
        joint_command_.q(i) =
            joint_command_prev_.q(i) + scale * position_increment;
        joint_command_.qdot(i) = scale * position_increment / dt;
      }
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                           1000,
                           "Scaled open-loop joint reference to enforce the "
                           "reference-error limit.");
    }
  }
  joint_command_prev_ = joint_command_;
  return joint_command_;
}

void TaskspaceControllerBase::write_joint_command(
    const KDL::JntArrayVel& command)
{
  if (has_position_command_interface_)
  {
    for (size_t joint_ind = 0; joint_ind < n_joints_; ++joint_ind)
    {
      command_interfaces_[joint_ind].set_value(command.q(joint_ind));
    }
  }

  if (has_velocity_command_interface_)
  {
    const auto velocity_offset =
        has_position_command_interface_ ? n_joints_ : 0;

    for (size_t joint_ind = 0; joint_ind < n_joints_; ++joint_ind)
    {
      command_interfaces_[velocity_offset + joint_ind].set_value(
          command.qdot(joint_ind));
    }
  }
}

void TaskspaceControllerBase::stop_motion()
{
  joint_command_prev_.qdot.data.setZero();
  write_joint_command(joint_command_prev_);
}

void TaskspaceControllerBase::write_zero_velocity_command()
{
  if (has_velocity_command_interface_)
  {
    const auto velocity_offset =
        has_position_command_interface_ ? n_joints_ : 0;
    for (size_t joint_ind = 0; joint_ind < n_joints_; ++joint_ind)
    {
      command_interfaces_[velocity_offset + joint_ind].set_value(0.0);
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
  if (!read_state_from_hardware(joint_state_))
  {
    return false;
  }

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

void TaskspaceControllerBase::publish_diagnostics(
    const rclcpp::Time& time, const KDL::JntArrayVel& joint_cmd,
    const KDL::JntArrayVel& joint_fb, const ctrl::Pose& pose_cmd,
    const ctrl::Pose& pose_fb)
{
  if (!rt_diagnostic_pub_ || !rt_diagnostic_pub_->trylock())
    return;

  ctrl::Vector3D trans_error, orient_error;
  ctrl::computePoseError(pose_cmd, pose_fb, trans_error, orient_error);

  auto& msg = rt_diagnostic_pub_->msg_;
  msg.header.stamp = time;

  Eigen::Map<ctrl::VectorND>(msg.command.position.data(), n_joints_) =
      joint_cmd.q.data;
  Eigen::Map<ctrl::VectorND>(msg.command.velocity.data(), n_joints_) =
      joint_cmd.qdot.data;
  Eigen::Map<ctrl::VectorND>(msg.state.position.data(), n_joints_) =
      joint_fb.q.data;
  Eigen::Map<ctrl::VectorND>(msg.state_error.position.data(), n_joints_) =
      joint_cmd.q.data - joint_fb.q.data;

  if (has_velocity_state_interface_)
  {
    Eigen::Map<ctrl::VectorND>(msg.state.velocity.data(), n_joints_) =
        joint_fb.qdot.data;
    Eigen::Map<ctrl::VectorND>(msg.state_error.velocity.data(), n_joints_) =
        joint_cmd.qdot.data - joint_fb.qdot.data;
  }

  msg.setpoint.pose = ctrl::transformEigenToROS(pose_cmd);
  msg.pose.pose = ctrl::transformEigenToROS(pose_fb);

  msg.position_error = ctrl::transformEigenToROS(trans_error);
  msg.position_error_norm = trans_error.norm();

  msg.orientation_error = ctrl::transformEigenToROS(orient_error);
  msg.orientation_error_norm = orient_error.norm();

  rt_diagnostic_pub_->unlockAndPublish();
}

}  // namespace taskspace_controllers
