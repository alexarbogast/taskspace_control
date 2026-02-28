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

#include "axially_symmetric_controllers/nullspace_controller.hpp"
#include "axially_symmetric_controllers/utility.hpp"

namespace axially_symmetric_controllers
{

controller_interface::return_type NullspaceController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  if (pose_param_listener_->is_old(pose_params_))
  {
    pose_params_ = pose_param_listener_->get_params();
  }

  read_state_from_hardware(joint_state_);
  const Setpoint* setpoint = setpoint_buffer_.readFromRT();

  KDL::Jacobian jac(n_joints_);
  robot_jacobian_solver_->JntToJac(joint_state_.q, jac);

  KDL::Frame pose_kdl;
  robot_fk_solver_->JntToCart(joint_state_.q, pose_kdl);

  // --- Error computation ---
  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R_fk(
      pose_kdl.M.data);
  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R_setpoint(
      setpoint->pose.M.data);

  ctrl::Vector3D aim_current(R_fk * tool_frame_axis_);
  ctrl::Vector3D aim_desired(R_setpoint * setpoint_frame_axis_);

  ctrl::Vector3D rot_axis = axisBetween(aim_current, aim_desired);
  double rot_angle = angleBetween(aim_current, aim_desired);

  ctrl::Vector2D orient_error(rot_axis.x(), rot_axis.y());
  orient_error *= rot_angle;

  ctrl::Vector3D pos_error((setpoint->pose.p - pose_kdl.p).data);

  ctrl::Vector5D cart_cmd;
  cart_cmd << pose_params_.k_position * pos_error + setpoint->twist.head<3>(),
      pose_params_.k_orient * orient_error;

  // --- Redundancy resolution ---
  ctrl::VectorND h = rr_objective_->getJointControlCmd(joint_state_);

  // --- Control law ---
  ctrl::MatrixND J = jac.data.block(0, 0, 5, n_joints_);
  ctrl::MatrixND J_pinv = ctrl::rightPinv(J);

  static ctrl::MatrixND I = ctrl::MatrixND::Identity(n_joints_, n_joints_);
  ctrl::VectorND joint_cmd = J_pinv * cart_cmd + (I - J_pinv * J) * h;

  ctrl::VectorND new_position =
      joint_state_.q.data + (joint_cmd * period.seconds());

  auto cmd = ctrl::transformEigenToKDL(new_position, joint_cmd);
  write_command(cmd);
  return controller_interface::return_type::OK;
}

}  // namespace axially_symmetric_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(axially_symmetric_controllers::NullspaceController,
                       controller_interface::ControllerInterface)
