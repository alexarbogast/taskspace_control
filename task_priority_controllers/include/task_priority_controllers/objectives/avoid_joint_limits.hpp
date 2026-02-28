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

#include "task_priority_controllers/objectives/objective_plugin.hpp"
#include "task_priority_controllers/avoid_joint_limits_parameters.hpp"

namespace task_priority_controllers
{

class AvoidJointLimits : public RRObjective
{
public:
  AvoidJointLimits() = default;

  virtual bool init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                    const KDL::Chain& chain,
                    const KDL::JntArray& upper_pos_limits,
                    const KDL::JntArray& lower_pos_limits) override;
  virtual ctrl::VectorND
  getJointControlCmd(const KDL::JntArrayVel& joint_state) override;

protected:
  std::shared_ptr<avoid_joint_limits::ParamListener> param_listener_;
  avoid_joint_limits::Params params_;

  KDL::JntArray limits_avg;
};

}  // namespace task_priority_controllers
