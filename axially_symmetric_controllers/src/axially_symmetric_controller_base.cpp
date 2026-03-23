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

#include "axially_symmetric_controllers/axially_symmetric_controller_base.hpp"

namespace axially_symmetric_controllers
{

controller_interface::CallbackReturn AxiallySymmetricControllerBase::on_init()
{
  // Initialize base class
  if (task_priority_controllers::PoseController::on_init() !=
      controller_interface::CallbackReturn::SUCCESS)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize the AxiallySymmetricController
  try
  {
    as_param_listener_ =
        std::make_shared<axially_symmetric_controller::ParamListener>(
            get_node());
  }
  catch (const std::exception& e)
  {
    fprintf(stderr,
            "Exception thrown during controller's init with message: %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AxiallySymmetricControllerBase::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  // Configure the base class
  if (task_priority_controllers::PoseController::on_configure(previous_state) !=
      controller_interface::CallbackReturn::SUCCESS)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Configure the AxiallySymmetricController
  as_params_ = as_param_listener_->get_params();
  auto& tf_axis = as_params_.eef_frame_axis;
  auto& sf_axis = as_params_.setpoint_frame_axis;

  tool_frame_axis_ = ctrl::Vector3D(tf_axis[0], tf_axis[1], tf_axis[2]);
  setpoint_frame_axis_ = ctrl::Vector3D(sf_axis[0], sf_axis[1], sf_axis[2]);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AxiallySymmetricControllerBase::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(get_node()->get_logger(),
              "Activating AxiallySymmetricController...");

  // Activate base class
  if (TaskspaceControllerBase::on_activate(previous_state) !=
      controller_interface::CallbackReturn::SUCCESS)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize joint state from hardware
  read_state_from_hardware(joint_state_);

  Setpoint fk;
  robot_fk_solver_->JntToCart(joint_state_.q, fk.pose);

  // Initialize a pose with the setpoint_frame_axis_ aiming
  // in the direction of the tool_frame_axis_
  ctrl::Matrix3D R_fk;
  ctrl::transformKDLToEigen(fk.pose.M, R_fk);
  ctrl::Vector3D a_target = R_fk * tool_frame_axis_;
  a_target.normalize();

  ctrl::Vector3D a_set = setpoint_frame_axis_;
  a_set.normalize();

  ctrl::Quaternion q_align = ctrl::Quaternion::FromTwoVectors(a_set, a_target);

  Eigen::AngleAxisd twist(0.0, a_target);
  ctrl::Quaternion q_final = twist * q_align;
  ctrl::Matrix3D R = q_final.toRotationMatrix();

  Setpoint init_setpoint;
  init_setpoint.pose.M = ctrl::transformEigenToKDL(R);
  init_setpoint.pose.p = fk.pose.p;  // keep same position

  setpoint_buffer_.writeFromNonRT(std::move(init_setpoint));
  return CallbackReturn::SUCCESS;
}

}  // namespace axially_symmetric_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    axially_symmetric_controllers::AxiallySymmetricControllerBase,
    controller_interface::ControllerInterface)
