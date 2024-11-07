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

#include <axially_symmetric_controllers/nullspace_controller.h>
#include <axially_symmetric_controllers/utility.h>

namespace axially_symmetric_controllers
{

bool NullspaceController::init(hardware_interface::PositionJointInterface* hw,
                               ros::NodeHandle& nh)
{
  taskspace_controllers::PoseController::init(hw, nh);
  task_priority_controllers::TaskPriorityController::init(hw, nh);
  return true;
}

void NullspaceController::update(const ros::Time&, const ros::Duration& period)
{
  synchronizeJointStates();  // update state

  const DynamicParams* params = dynamic_params_.readFromRT();
  const Setpoint* setpoint = setpoint_.readFromRT();

  KDL::Jacobian jac(n_joints_);
  robot_jacobian_solver_->JntToJac(robot_state_.q, jac);

  KDL::Frame pose;
  robot_fk_solver_->JntToCart(robot_state_.q, pose);

  /* error */
  ctrl::Vector3D aim_current(pose.M.UnitZ().data);
  ctrl::Vector3D aim_desired(setpoint->pose.M.UnitZ().data);

  ctrl::Vector3D rot_axis = axisBetween(aim_current, aim_desired);
  double rot_angle = angleBetween(aim_current, aim_desired);

  ctrl::Vector2D orient_error(rot_axis.x(), rot_axis.y());
  orient_error *= rot_angle;

  ctrl::Vector3D pos_error((setpoint->pose.p - pose.p).data);

  ctrl::Vector5D cart_cmd;
  cart_cmd << params->k_position * pos_error + setpoint->twist.head<3>(),
      params->k_orient * orient_error;

  /* redundancy resolution */
  ctrl::VectorND h = rr_objective_->getJointControlCmd(robot_state_);

  /* control */
  ctrl::MatrixND J = jac.data.block(0, 0, 5, n_joints_);
  ctrl::MatrixND J_pinv = ctrl::rightPinv(J);

  static ctrl::MatrixND I = ctrl::MatrixND::Identity(n_joints_, n_joints_);
  ctrl::VectorND joint_cmd = J_pinv * cart_cmd + (I - J_pinv * jac.data) * h;

  ctrl::VectorND new_position =
      robot_state_.q.data + (joint_cmd * period.toSec());
  writeCommand(new_position);
}

}  // namespace axially_symmetric_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(axially_symmetric_controllers::NullspaceController,
                       controller_interface::ControllerBase)
