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
#include "task_priority_controllers/maximize_manipulability_parameters.hpp"

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>

#include <memory>

namespace task_priority_controllers
{

class MaximizeManipulability : public RRObjective
{
public:
  MaximizeManipulability() = default;

  virtual bool init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                    const KDL::Chain& chain,
                    const KDL::JntArray& upper_pos_limits,
                    const KDL::JntArray& lower_pos_limits) override;
  virtual ctrl::VectorND
  getJointControlCmd(const KDL::JntArrayVel& joint_state) override;

protected:
  std::shared_ptr<maximize_manipulability::ParamListener> param_listener_;
  maximize_manipulability::Params params_;

  std::unique_ptr<KDL::ChainJntToJacSolver> robot_jacobian_solver_;
  std::unique_ptr<KDL::ChainJntToJacDotSolver> robot_jacobian_dot_solver_;
};

}  // namespace task_priority_controllers
