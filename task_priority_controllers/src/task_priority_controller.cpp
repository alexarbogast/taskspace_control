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

#include <task_priority_controllers/task_priority_controller.h>

namespace task_priority_controllers
{

bool TaskPriorityController::init(
    hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh)
{
  Base::init(hw, nh);

  // Load redundancy resolution objective
  std::string objective_type = "minimize_velocity";
  nh.getParam("rr_objective_type", objective_type);

  rr_objective_loader_ = std::make_unique<pluginlib::ClassLoader<RRObjective>>(
      "task_priority_controllers", "task_priority_controllers::RRObjective");

  try
  {
    rr_objective_ = rr_objective_loader_->createUniqueInstance(objective_type);
  }
  catch (const pluginlib::PluginlibException& e)
  {
    ROS_ERROR_STREAM(
        "Failed to load redundancy resolution plugin. Execption: " << e.what());
    return false;
  }

  return rr_objective_->init(nh, robot_chain_, upper_pos_limits_,
                             lower_pos_limits_);
}

}  // namespace task_priority_controllers
