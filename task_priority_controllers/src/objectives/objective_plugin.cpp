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

#include "task_priority_controllers/objectives/objective_plugin.hpp"

namespace task_priority_controllers
{

bool RRObjective::init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                       const KDL::Chain& chain,
                       const std::vector<ctrl::JointLimits>& joint_limits)
{
  robot_chain_ = chain;
  n_joints_ = robot_chain_.getNrOfJoints();

  joint_limits_ = joint_limits;
  return true;
}

}  // namespace task_priority_controllers
