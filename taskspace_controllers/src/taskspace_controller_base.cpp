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

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace taskspace_controllers
{

bool TaskspaceControllerBase::init(
    hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh)
{
  if (initialized_)
    return true;

  const std::string ns = nh.getNamespace();

  // Load robot description and link names
  std::string robot_description;
  if (!ros::param::search("robot_description", robot_description))
  {
    ROS_ERROR("robot_description not found in enclosing namespaces");
    return false;
  }
  if (!nh.getParam(robot_description, robot_description))
  {
    ROS_ERROR_STREAM("Failed to load " << robot_description
                                       << " from parameter server");
    return false;
  }
  if (!nh.getParam("base_link", base_link_))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/base_link"
                                       << " from parameter server");
    return false;
  }
  if (!nh.getParam("eef_link", eef_link_))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/eef_link"
                                       << " from parameter server");
    return false;
  }

  // Initialize kinematic model
  urdf::Model urdf_model;
  KDL::Tree kdl_tree;
  if (!urdf_model.initString(robot_description))
  {
    ROS_ERROR("Failed to initialize urdf model from 'robot_description'");
    return false;
  }
  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree))
  {
    ROS_FATAL("Failed to parse KDL tree from  urdf model");
    return false;
  }
  if (!kdl_tree.getChain(base_link_, eef_link_, robot_chain_))
  {
    ROS_FATAL_STREAM("Failed to build kinematic chain from '"
                     << base_link_ << "' to '" << eef_link_
                     << "'. Make sure these links exist in the URDF.");
    return false;
  }

  // Parse joint limits
  std::vector<std::string> joint_names;
  if (!nh.getParam("joints", joint_names))
  {
    ROS_ERROR_STREAM("Failed to load " << ns << "/joints"
                                       << " from parameter server");
    return false;
  }
  n_joints_ = joint_names.size();

  upper_pos_limits_.resize(n_joints_);
  lower_pos_limits_.resize(n_joints_);
  for (size_t i = 0; i < n_joints_; ++i)
  {
    if (!urdf_model.getJoint(joint_names[i]))
    {
      ROS_ERROR_STREAM("Joint " + joint_names[i] + " does not exist in URDF");
      return false;
    }
    if (urdf_model.getJoint(joint_names[i])->type == urdf::Joint::CONTINUOUS)
    {
      upper_pos_limits_(i) = std::nan("0");
      lower_pos_limits_(i) = std::nan("0");
    }
    else
    {
      upper_pos_limits_(i) = urdf_model.getJoint(joint_names[i])->limits->upper;
      lower_pos_limits_(i) = urdf_model.getJoint(joint_names[i])->limits->lower;
    }
  }

  // Get joint handles from hardware interface
  joint_handles_.resize(n_joints_);
  for (size_t i = 0; i < n_joints_; ++i)
  {
    try
    {
      joint_handles_[i] = hw->getHandle(joint_names[i]);
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception getting joint handles from hw interface"
                       << e.what());
      return false;
    }
  }

  robot_fk_solver_ =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(robot_chain_);

  robot_state_.resize(n_joints_);

  // Find setpoint topic
  if (!nh.getParam("setpoint_topic", setpoint_topic_))
  {
    ROS_ERROR_STREAM("Failed to load " << ns << "/setpoint_topic"
                                       << " from parameter server");
    return false;
  }

  // Services
  query_pose_service_ = nh.advertiseService(
      "query_pose", &TaskspaceControllerBase::queryPoseService, this);

  initialized_ = true;
  return true;
}

void TaskspaceControllerBase::synchronizeJointStates()
{
  // Synchronize the internal state with the hardware
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    robot_state_.q(i) = joint_handles_[i].getPosition();
    robot_state_.qdot(i) = joint_handles_[i].getVelocity();
  }
}

void TaskspaceControllerBase::writeCommand(const ctrl::VectorND& cmd)
{
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    joint_handles_[i].setCommand(cmd[i]);
  }
}

bool TaskspaceControllerBase::queryPoseService(
    taskspace_control_msgs::QueryPose::Request& req,
    taskspace_control_msgs::QueryPose::Response& resp)
{
  KDL::JntArray robot_state(n_joints_);
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    robot_state(i) = joint_handles_[i].getPosition();
  }

  KDL::Frame pose;
  robot_fk_solver_->JntToCart(robot_state, pose);

  resp.pose.position.x = pose.p.x();
  resp.pose.position.y = pose.p.y();
  resp.pose.position.z = pose.p.z();

  pose.M.GetQuaternion(resp.pose.orientation.x, resp.pose.orientation.y,
                       resp.pose.orientation.z, resp.pose.orientation.w);
  return true;
}

}  // namespace taskspace_controllers
