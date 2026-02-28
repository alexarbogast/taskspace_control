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

#include "task_priority_controllers/task_priority_controller.hpp"

namespace task_priority_controllers
{

controller_interface::CallbackReturn TaskPriorityController::on_init()
{
  // Initialize base class
  if (Base::on_init() != controller_interface::CallbackReturn::SUCCESS)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize the TaskPriorityController
  try
  {
    tp_param_listener_ =
        std::make_shared<task_priority_controllers::ParamListener>(get_node());
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

controller_interface::CallbackReturn TaskPriorityController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  auto node = get_node();
  RCLCPP_INFO(node->get_logger(), "Configuring TaskPriorityController...");

  tp_params_ = tp_param_listener_->get_params();

  if (Base::on_configure(previous_state) !=
      controller_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize base controller.");
    return CallbackReturn::FAILURE;
  }

  rr_objective_loader_ = std::make_unique<pluginlib::ClassLoader<RRObjective>>(
      "task_priority_controllers", "task_priority_controllers::RRObjective");

  try
  {
    rr_objective_ = rr_objective_loader_->createUniqueInstance(
        tp_params_.rr_objective_type);
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "\033[32mLoaded RR Objective: \033[0m"
                       "\033[1;32m"
                           << tp_params_.rr_objective_type << "\033[0m");
  }
  catch (const pluginlib::PluginlibException& e)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to load redundancy resolution plugin. Execption: %s",
                 e.what());
    return CallbackReturn::FAILURE;
  }

  if (!rr_objective_->init(node, robot_chain_, upper_pos_limits_,
                           lower_pos_limits_))
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to initialize redundancy resolution objective.");
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

}  // namespace task_priority_controllers
