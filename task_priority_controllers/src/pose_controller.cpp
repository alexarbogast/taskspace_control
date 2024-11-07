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

#include <task_priority_controllers/pose_controller.h>

namespace task_priority_controllers
{

bool PoseController::init(hardware_interface::PositionJointInterface* hw,
                          ros::NodeHandle& nh)
{
  taskspace_controllers::PoseController::init(hw, nh);
  TaskPriorityController::init(hw, nh);
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

  /* redundancy resolution */
  ctrl::VectorND h = rr_objective_->getJointControlCmd(robot_state_);

  /* control */
  static ctrl::MatrixND I = ctrl::MatrixND::Identity(n_joints_, n_joints_);
  ctrl::MatrixND J_pinv = ctrl::rightPinv(jac.data);
  ctrl::VectorND joint_cmd = J_pinv * cart_cmd + (I - J_pinv * jac.data) * h;

  ctrl::VectorND new_position =
      robot_state_.q.data + (joint_cmd * period.toSec());
  writeCommand(new_position);
}

}  // namespace task_priority_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_priority_controllers::PoseController,
                       controller_interface::ControllerBase)
