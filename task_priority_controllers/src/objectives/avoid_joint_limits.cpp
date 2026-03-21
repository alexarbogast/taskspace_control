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

#include "task_priority_controllers/objectives/avoid_joint_limits.hpp"
#include "task_priority_controllers/avoid_joint_limits_parameters.hpp"

namespace task_priority_controllers
{

bool AvoidJointLimits::init(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const KDL::Chain& chain, const std::vector<ctrl::JointLimits>& joint_limits)
{
  if (!RRObjective::init(node, chain, joint_limits))
  {
    return false;
  }

  for (size_t i = 0; i < n_joints_; ++i)
  {
    const auto& limits = joint_limits[i];
    if (std::isnan(limits.min_position) || std::isnan(limits.max_position))
    {
      RCLCPP_ERROR(node->get_logger(),
                   "Joint at index %lu has invalid position limits (NaN).", i);
      return false;
    }

    limits_avg(i) = 0.5 * (limits.min_position + limits.max_position);
  }

  try
  {
    param_listener_ = std::make_shared<avoid_joint_limits::ParamListener>(node);
  }
  catch (const std::exception& e)
  {
    fprintf(stderr,
            "Exception thrown during rr objective init with message: %s \n",
            e.what());
    return false;
  }

  return true;
}

ctrl::VectorND
AvoidJointLimits::getJointControlCmd(const KDL::JntArrayVel& joint_state)
{
  params_ = param_listener_->get_params();
  return params_.k_limits * (limits_avg.data - joint_state.q.data);
}

}  // namespace task_priority_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(task_priority_controllers::AvoidJointLimits,
                       task_priority_controllers::RRObjective)
