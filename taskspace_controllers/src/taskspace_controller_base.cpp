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

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "urdf/model.h"

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

  // use only position feedback for now
  const std::string interface = "position";
  for (const auto& joint : joint_names_)
  {
    cfg.names.push_back(joint + std::string("/").append(interface));
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

  has_position_command_interface_ = ctrl::contains_interface_type(
      params_.command_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_command_interface_ = ctrl::contains_interface_type(
      params_.command_interfaces, hardware_interface::HW_IF_VELOCITY);

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

    ctrl::JointLimits limits;
    if (j->type == urdf::Joint::CONTINUOUS)
    {
      // No position limits, but velocity may exist
      if (j->limits)
      {
        limits.max_velocity = j->limits->velocity;
      }
    }
    else if (j->limits)
    {
      limits.min_position = j->limits->lower;
      limits.max_position = j->limits->upper;
      limits.max_velocity = j->limits->velocity;
    }
    else
    {
      RCLCPP_WARN(logger, "Joint %s has no limits; using NaN.", jn.c_str());
    }

    joint_limits_[i] = limits;
  }

  robot_fk_solver_ =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(robot_chain_);

  // Create service for query_pose
  query_pose_service_ = get_node()->create_service<QueryPose>(
      get_node()->get_name() + std::string("/query_pose"),
      std::bind(&TaskspaceControllerBase::queryPoseServiceCb, this,
                std::placeholders::_1, std::placeholders::_2));

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
  size_t pos_ind = 0;
  for (size_t joint_ind = 0; joint_ind < n_joints_; ++joint_ind)
  {
    state.q(joint_ind) =
        state_interfaces_[pos_ind * n_joints_ + joint_ind].get_value();
    nan_position |= std::isnan(state.q(joint_ind));
  }

  if (nan_position)
  {
    state.q = last_commanded_.q;
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
    const auto& limits = joint_limits_[joint_ind];

    double q_cmd = cmd.q(joint_ind);
    double qdot_cmd = cmd.qdot(joint_ind);

    // Clamp output command
    if (!std::isnan(limits.min_position) && !std::isnan(limits.max_position))
    {
      q_cmd = std::clamp(q_cmd, limits.min_position, limits.max_position);
    }
    if (!std::isnan(limits.max_velocity))
    {
      qdot_cmd =
          std::clamp(qdot_cmd, -limits.max_velocity, limits.max_velocity);
    }

    // Write output command to hardware
    if (has_position_command_interface_)
    {
      command_interfaces_[pos_ind * n_joints_ + joint_ind].set_value(q_cmd);
    }
    if (has_velocity_command_interface_)
    {
      command_interfaces_[vel_ind * n_joints_ + joint_ind].set_value(qdot_cmd);
    }

    last_commanded_.q(joint_ind) = q_cmd;
    last_commanded_.qdot(joint_ind) = qdot_cmd;
  }
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

}  // namespace taskspace_controllers
