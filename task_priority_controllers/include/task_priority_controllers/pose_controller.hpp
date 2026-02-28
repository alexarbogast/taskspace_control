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

#pragma once

#include "task_priority_controllers/task_priority_controller.hpp"
#include "taskspace_controllers/pose_controller.hpp"

namespace task_priority_controllers
{

class PoseController : public TaskPriorityController,
                       public taskspace_controllers::PoseController
{
public:
  virtual controller_interface::CallbackReturn on_init() override;

  virtual controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;

  virtual controller_interface::return_type
  update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
};

}  // namespace task_priority_controllers
