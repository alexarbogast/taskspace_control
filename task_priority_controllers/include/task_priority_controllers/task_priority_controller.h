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

#include <taskspace_controllers/taskspace_controller_base.h>
#include <task_priority_controllers/objectives/objective_plugin.h>

#include <pluginlib/class_loader.h>

namespace task_priority_controllers
{

class TaskPriorityController
  : public virtual taskspace_controllers::TaskspaceControllerBase
{
public:
  virtual bool init(hardware_interface::PositionJointInterface* hw,
                    ros::NodeHandle& nh) override;

protected:
  typedef taskspace_controllers::TaskspaceControllerBase Base;

  // redundancy resolution objective
  std::shared_ptr<RRObjective> rr_objective_;
  std::unique_ptr<pluginlib::ClassLoader<RRObjective>> rr_objective_loader_;
};

}  // namespace task_priority_controllers
