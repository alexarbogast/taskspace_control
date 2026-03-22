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

#include "axially_symmetric_controllers/twist_decomposition_controller.hpp"
#include "axially_symmetric_controllers/utility.hpp"

namespace axially_symmetric_controllers
{

controller_interface::return_type TwistDecompositionController::update(
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

  // Safety: bail out near singularities
  if (!check_manipulability(jac))
  {
    return controller_interface::return_type::OK;
  }

  KDL::Frame pose_kdl;
  robot_fk_solver_->JntToCart(joint_state_.q, pose_kdl);

  // --- Error computation ---
  ctrl::Matrix3D R_fk, R_setpoint;
  ctrl::transformKDLToEigen(pose_kdl.M, R_fk);
  ctrl::transformKDLToEigen(setpoint->pose.M, R_setpoint);

  ctrl::Vector3D aim_current(R_fk * tool_frame_axis_);
  ctrl::Vector3D aim_desired(R_setpoint * setpoint_frame_axis_);

  ctrl::Vector3D rot_axis = axisBetween(aim_current, aim_desired);
  double rot_angle = angleBetween(aim_current, aim_desired);

  ctrl::Vector2D orient_error(rot_axis.x(), rot_axis.y());
  orient_error *= rot_angle;

  ctrl::Vector3D pos_error((setpoint->pose.p - pose_kdl.p).data);

  ctrl::Vector6D cart_cmd;
  cart_cmd << pose_params_.k_position * pos_error + setpoint->twist.head<3>(),
      pose_params_.k_orient * orient_error;

  // --- Redundancy resolution ---
  ctrl::VectorND h = rr_objective_->getJointControlCmd(joint_state_);

  // --- Twist decomposition ---
  ctrl::Vector3D e(pose_kdl.M.UnitZ().data);
  ctrl::Matrix3D eeT = e * e.transpose();

  ctrl::Matrix6D T = ctrl::Matrix6D::Zero();
  T.block<3, 3>(0, 0) = ctrl::Matrix3D::Identity();
  T.block<3, 3>(3, 3) = ctrl::Matrix3D::Identity() - eeT;

  ctrl::Vector3D perp_cmd = eeT * jac.data.block(3, 0, 3, n_joints_) * h;

  // --- Control ---
  ctrl::MatrixND J = jac.data;
  ctrl::MatrixND J_pinv = ctrl::rightPinv(J);

  ctrl::Vector6D mod_cart_cmd = T * cart_cmd;
  mod_cart_cmd.block<3, 1>(3, 0) += perp_cmd;

  ctrl::VectorND joint_cmd = J_pinv * mod_cart_cmd;

  KDL::JntArray q_cmd = ctrl::transformEigenToKDL(joint_cmd);
  auto cmd = create_command(joint_state_.q, q_cmd, period.seconds());
  write_command(cmd);

  return controller_interface::return_type::OK;
}

}  // namespace axially_symmetric_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    axially_symmetric_controllers::TwistDecompositionController,
    controller_interface::ControllerInterface)
