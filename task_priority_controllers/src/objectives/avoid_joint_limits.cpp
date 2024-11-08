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

#include <task_priority_controllers/objectives/avoid_joint_limits.h>

namespace task_priority_controllers
{

static double MANIP_THRESHOLD = 1e-10;

bool AvoidJointLimits::init(ros::NodeHandle& nh, const KDL::Chain& chain,
                            const KDL::JntArray& upper_pos_limits,
                            const KDL::JntArray& lower_pos_limits)
{
  RRObjective::init(nh, chain, upper_pos_limits, lower_pos_limits);

  limits_avg.data = (upper_pos_limits.data + lower_pos_limits.data) / 2;

  // Dynamic reconfigure
  dyn_reconf_server_ =
      std::make_shared<ReconfigureServer>(ros::NodeHandle(nh, "rr_objective"));
  dyn_reconf_server_->setCallback(std::bind(&AvoidJointLimits::reconfCallback,
                                            this, std::placeholders::_1,
                                            std::placeholders::_2));
  return true;
}

ctrl::VectorND
AvoidJointLimits::getJointControlCmd(const KDL::JntArrayVel& joint_state)
{
  const DynamicParams* params = dynamic_params_.readFromRT();
  return params->k_limits * (limits_avg.data - joint_state.q.data);
}

void AvoidJointLimits::reconfCallback(ObjectiveConfig& config,
                                      uint16_t /*level*/)
{
  DynamicParams dynamic_params;
  dynamic_params.k_limits = config.k_limits;
  dynamic_params_.writeFromNonRT(dynamic_params);
}

}  // namespace task_priority_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_priority_controllers::AvoidJointLimits,
                       task_priority_controllers::RRObjective)
