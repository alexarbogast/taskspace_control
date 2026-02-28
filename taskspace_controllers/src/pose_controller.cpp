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

#include "taskspace_controllers/pose_controller.hpp"
#include "taskspace_controllers/utility.hpp"

namespace taskspace_controllers
{

using namespace std::chrono_literals;

controller_interface::CallbackReturn PoseController::on_init()
{
  // Initialize base class
  if (TaskspaceControllerBase::on_init() !=
      controller_interface::CallbackReturn::SUCCESS)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize the PoseController
  try
  {
    pose_param_listener_ =
        std::make_shared<pose_controller::ParamListener>(get_node());
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
PoseController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  auto node = get_node();
  RCLCPP_INFO(node->get_logger(), "Configuring PoseController...");

  pose_params_ = pose_param_listener_->get_params();

  // Initialize base kinematics and joint info
  if (TaskspaceControllerBase::on_configure(previous_state) !=
      controller_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize base controller.");
    return CallbackReturn::FAILURE;
  }

  robot_jacobian_solver_ =
      std::make_unique<KDL::ChainJntToJacSolver>(robot_chain_);

  // --- Setpoint subscription ---
  setpoint_subscriber_ =
      node->create_subscription<taskspace_control_msgs::msg::PoseTwistSetpoint>(
          node->get_name() + std::string("/") + pose_params_.setpoint_topic, 1,
          std::bind(&PoseController::setpointCallback, this,
                    std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(), "PoseController configured for %u joints.",
              n_joints_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PoseController::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(get_node()->get_logger(), "Activating PoseController...");

  // Activate base class
  if (TaskspaceControllerBase::on_activate(previous_state) !=
      controller_interface::CallbackReturn::SUCCESS)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // initialize joint state from hardware
  read_state_from_hardware(joint_state_);

  Setpoint init_setpoint;
  robot_fk_solver_->JntToCart(joint_state_.q, init_setpoint.pose);
  setpoint_buffer_.writeFromNonRT(std::move(init_setpoint));
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

  // --- Control law ---
  ctrl::VectorND joint_cmd = ctrl::rightPinv(jac.data) * cart_cmd;
  ctrl::VectorND new_position =
      joint_state_.q.data + (joint_cmd * period.seconds());

  auto cmd = ctrl::transformEigenToKDL(new_position, joint_cmd);
  write_command(cmd);
  return controller_interface::return_type::OK;
}

void PoseController::setpointCallback(
    const std::shared_ptr<taskspace_control_msgs::msg::PoseTwistSetpoint> msg)
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
  setpoint_buffer_.writeFromNonRT(setpoint);
}

}  // namespace taskspace_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(taskspace_controllers::PoseController,
                       controller_interface::ControllerInterface)
