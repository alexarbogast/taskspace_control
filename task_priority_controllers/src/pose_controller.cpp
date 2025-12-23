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

#include <task_priority_controllers/pose_controller.hpp>
#include <taskspace_controllers/utility.hpp>

#include <kdl/jacobian.hpp>

namespace task_priority_controllers
{

controller_interface::CallbackReturn PoseController::on_init()
{
  if (taskspace_controllers::PoseController::on_init() !=
          controller_interface::CallbackReturn::SUCCESS ||
      TaskPriorityController::on_init() !=
          controller_interface::CallbackReturn::SUCCESS)
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PoseController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  auto node = get_node();
  RCLCPP_INFO(node->get_logger(), "Configuring PoseController...");

  if (taskspace_controllers::PoseController::on_configure(previous_state) !=
      controller_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to initialize base pose controller.");
    return CallbackReturn::FAILURE;
  }

  if (TaskPriorityController::on_configure(previous_state) !=
      controller_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to initialize task-priority controller.");
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type PoseController::update(
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

  ctrl::Pose pose;
  ctrl::transformKDLToEigen(pose_kdl, pose);

  ctrl::Pose sp_pose;
  ctrl::transformKDLToEigen(setpoint->pose, sp_pose);

  // --- Error computation ---
  ctrl::AngleAxis aa(sp_pose.rotation() * pose.rotation().inverse());
  ctrl::Vector3D orient_error = aa.axis() * aa.angle();
  ctrl::Vector3D trans_error(sp_pose.translation() - pose.translation());

  ctrl::Vector6D cart_cmd;
  cart_cmd << pose_params_.k_position * trans_error,
      pose_params_.k_orient * orient_error;
  cart_cmd += setpoint->twist;

  // --- Redundancy resolution ---
  ctrl::VectorND h = rr_objective_->getJointControlCmd(joint_state_);

  // --- Control law ---
  static ctrl::MatrixND I = ctrl::MatrixND::Identity(n_joints_, n_joints_);
  ctrl::MatrixND J_pinv = ctrl::rightPinv(jac.data);
  ctrl::VectorND joint_cmd = J_pinv * cart_cmd + (I - J_pinv * jac.data) * h;

  ctrl::VectorND new_position =
      joint_state_.q.data + (joint_cmd * period.seconds());

  auto cmd = ctrl::transformEigenToKDL(new_position, joint_cmd);
  write_command(cmd);
  return controller_interface::return_type::OK;
}

}  // namespace task_priority_controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(task_priority_controllers::PoseController,
                       controller_interface::ControllerInterface)
