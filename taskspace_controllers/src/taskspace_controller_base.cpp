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

#include <taskspace_controllers/taskspace_controller_base.h>
#include <controller_interface/helpers.hpp>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace taskspace_controllers
{

controller_interface::InterfaceConfiguration
TaskspaceControllerBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(params_.joints.size() * params_.command_interfaces.size());
  for (const auto& type : params_.command_interfaces)
  {
    for (const auto& joint : params_.joints)
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
  for (const auto& joint : params_.joints)
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

  if (params_.joints.empty())
  {
    RCLCPP_ERROR(logger, "'joints' parameter was empty");
    return controller_interface::CallbackReturn::FAILURE;
  }
  n_joints_ = params_.joints.size();

  // allocate dynamic memory
  last_reference_.resize(n_joints_);
  last_commanded_ = last_reference_;
  joint_state_ = last_reference_;

  if (params_.command_interfaces.empty())
  {
    RCLCPP_ERROR(logger, "'command_interfaces' parameter was empty");
    return controller_interface::CallbackReturn::FAILURE;
  }

  joint_command_handles_.resize(allowed_interface_types_.size());
  for (auto& itf : joint_command_handles_)
  {
    itf.reserve(params_.joints.size());
  }

  has_position_command_interface_ = contains_interface_type(
      params_.command_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_command_interface_ = contains_interface_type(
      params_.command_interfaces, hardware_interface::HW_IF_VELOCITY);

  joint_state_handles_.resize(allowed_interface_types_.size());

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

  upper_pos_limits_.resize(n_joints_);
  lower_pos_limits_.resize(n_joints_);
  for (size_t i = 0; i < n_joints_; ++i)
  {
    const auto& jn = params_.joints[i];
    auto j = urdf_model.getJoint(jn);
    if (!j)
    {
      RCLCPP_ERROR(logger, "Joint '%s' not found in URDF.", jn.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    if (j->type == urdf::Joint::CONTINUOUS)
    {
      upper_pos_limits_(i) = std::numeric_limits<double>::quiet_NaN();
      lower_pos_limits_(i) = std::numeric_limits<double>::quiet_NaN();
    }
    else if (j->limits)
    {
      upper_pos_limits_(i) = j->limits->upper;
      lower_pos_limits_(i) = j->limits->lower;
    }
    else
    {
      RCLCPP_WARN(logger, "Joint %s has no limits; using NaN.", jn.c_str());
      upper_pos_limits_(i) = std::numeric_limits<double>::quiet_NaN();
      lower_pos_limits_(i) = std::numeric_limits<double>::quiet_NaN();
    }
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

  for (const auto& interface : params_.command_interfaces)
  {
    auto it = std::find(allowed_interface_types_.begin(),
                        allowed_interface_types_.end(), interface);
    auto index = static_cast<size_t>(
        std::distance(allowed_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
            command_interfaces_, params_.joints, interface,
            joint_command_handles_[index]))
    {
      RCLCPP_ERROR(logger, "Expected %u '%s' command interfaces, got %zu.",
                   n_joints_, interface.c_str(),
                   joint_command_handles_[index].size());
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(logger, "Activated TaskspaceControllerBase");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TaskspaceControllerBase::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  // release loaned interfaces if needed (framework often handles this)
  joint_command_handles_.clear();
  joint_state_handles_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TaskspaceControllerBase::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // derived classes will compute command vector and call write_command()
  return controller_interface::return_type::OK;
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

KDL::JntArrayVel TaskspaceControllerBase::create_kdl_state(
    const ctrl::VectorND& q, const ctrl::VectorND& qdot)
{
  const size_t n = q.size();
  KDL::JntArrayVel out(n);

  Eigen::Map<Eigen::VectorXd>(out.q.data.data(), n) = q;
  Eigen::Map<Eigen::VectorXd>(out.qdot.data.data(), n) = qdot;

  return out;
}

bool TaskspaceControllerBase::contains_interface_type(
    const std::vector<std::string>& interface_type_list,
    const std::string& interface_type)
{
  return std::find(interface_type_list.begin(), interface_type_list.end(),
                   interface_type) != interface_type_list.end();
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
