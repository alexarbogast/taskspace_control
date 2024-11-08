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

#include <task_priority_controllers/objectives/objective_plugin.h>

#include <realtime_tools/realtime_buffer.h>
#include <dynamic_reconfigure/server.h>
#include <task_priority_controllers/JointLimitsObjectiveConfig.h>

namespace task_priority_controllers
{

class AvoidJointLimits : public RRObjective
{
public:
  AvoidJointLimits() = default;

  virtual bool init(ros::NodeHandle& nh, const KDL::Chain& chain,
                    const KDL::JntArray& upper_pos_limits,
                    const KDL::JntArray& lower_pos_limits) override;
  virtual ctrl::VectorND
  getJointControlCmd(const KDL::JntArrayVel& joint_state) override;

protected:
  typedef JointLimitsObjectiveConfig ObjectiveConfig;
  typedef dynamic_reconfigure::Server<ObjectiveConfig> ReconfigureServer;

  void reconfCallback(ObjectiveConfig& config, uint16_t /*level*/);

  KDL::JntArray limits_avg;

  // dynamic reconfigure
  struct DynamicParams
  {
    DynamicParams() = default;
    double k_limits = 1.0;
  };
  realtime_tools::RealtimeBuffer<DynamicParams> dynamic_params_;
  std::shared_ptr<ReconfigureServer> dyn_reconf_server_;
};

}  // namespace task_priority_controllers
