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

#include <task_priority_controllers/objectives/minimize_velocity.h>

namespace task_priority_controllers
{

ctrl::VectorND MinimizeVelocity::getJointControlCmd()
{
  return ctrl::VectorND::Zero(n_joints_);
}

}  // namespace task_priority_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_priority_controllers::MinimizeVelocity,
                       task_priority_controllers::RRObjective)
