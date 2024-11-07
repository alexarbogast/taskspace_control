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

#include <taskspace_controllers/pose_controller.h>

#include "kdl/jacobian.hpp"
#include "kdl/frames.hpp"
#include "taskspace_controllers/utility.h"

#include <pluginlib/class_list_macros.h>

namespace taskspace_controllers
{

bool PoseController::init(hardware_interface::PositionJointInterface* hw,
                          ros::NodeHandle& nh)
{
  Base::init(hw, nh);

  robot_jacobian_solver_ =
      std::make_unique<KDL::ChainJntToJacSolver>(robot_chain_);

  sub_setpoint_ =
      nh.subscribe(setpoint_topic_, 1, &PoseController::setpointCallback, this);

  // Dynamic reconfigure
  dyn_reconf_server_ = std::make_shared<ReconfigureServer>(nh);
  dyn_reconf_server_->setCallback(std::bind(&PoseController::reconfCallback,
                                            this, std::placeholders::_1,
                                            std::placeholders::_2));
  return true;
}

void PoseController::update(const ros::Time&, const ros::Duration& period)
{
  synchronizeJointStates();  // update state

  const DynamicParams* params = dynamic_params_.readFromRT();
  const Setpoint* setpoint = setpoint_.readFromRT();

  KDL::Jacobian jac(n_joints_);
  robot_jacobian_solver_->JntToJac(robot_state_.q, jac);

  KDL::Frame pose;
  robot_fk_solver_->JntToCart(robot_state_.q, pose);

  /* error */
  ctrl::Quaternion current_q, setpoint_q;
  pose.M.GetQuaternion(current_q.x(), current_q.y(), current_q.z(),
                       current_q.w());
  setpoint->pose.M.GetQuaternion(setpoint_q.x(), setpoint_q.y(), setpoint_q.z(),
                                 setpoint_q.w());

  ctrl::Vector3D orient_error = (setpoint_q * current_q.inverse()).vec();
  ctrl::Vector3D trans_error((setpoint->pose.p - pose.p).data);

  ctrl::Vector6D cart_cmd;
  cart_cmd << params->k_position * trans_error, params->k_orient * orient_error;
  cart_cmd += setpoint->twist;

  /* control */
  ctrl::VectorND joint_cmd = ctrl::rightPinv(jac.data) * cart_cmd;
  ctrl::VectorND new_position =
      robot_state_.q.data + (joint_cmd * period.toSec());
  writeCommand(new_position);
}

void PoseController::starting(const ros::Time&)
{
  synchronizeJointStates();

  Setpoint init_setpoint;
  robot_fk_solver_->JntToCart(robot_state_.q, init_setpoint.pose);
  setpoint_.initRT(init_setpoint);
}

void PoseController::stopping(const ros::Time&) {}

void PoseController::reconfCallback(ControllerConfig& config,
                                    uint16_t /*level*/)
{
  DynamicParams dynamic_params;
  dynamic_params.k_position = config.k_position;
  dynamic_params.k_orient = config.k_orient;

  dynamic_params_.writeFromNonRT(dynamic_params);
}

void PoseController::setpointCallback(
    const taskspace_control_msgs::PoseTwistSetpointConstPtr& msg)
{
  Setpoint setpoint;
  setpoint.pose.p = KDL::Vector(msg->pose.position.x, msg->pose.position.y,
                                msg->pose.position.z);

  setpoint.pose.M = KDL::Rotation::Quaternion(
      msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
      msg->pose.orientation.w);

  setpoint.twist << msg->twist.linear.x, msg->twist.linear.y,
      msg->twist.linear.z, msg->twist.angular.x, msg->twist.angular.y,
      msg->twist.angular.z;

  setpoint_.writeFromNonRT(setpoint);
}

}  // namespace taskspace_controllers

PLUGINLIB_EXPORT_CLASS(taskspace_controllers::PoseController,
                       controller_interface::ControllerBase)
