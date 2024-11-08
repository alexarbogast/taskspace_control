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

#include <ros/ros.h>
#include <taskspace_controllers/utility.h>

#include <kdl/chain.hpp>
#include <kdl/jntarrayvel.hpp>

namespace task_priority_controllers
{
class RRObjective
{
public:
  RRObjective() = default;

  virtual bool init(ros::NodeHandle& nh, const KDL::Chain& chain,
                    const KDL::JntArray& upper_pos_limits,
                    const KDL::JntArray& lower_pos_limits);
  virtual ctrl::VectorND
  getJointControlCmd(const KDL::JntArrayVel& joint_state) = 0;

protected:
  unsigned int n_joints_;
  KDL::Chain robot_chain_;

  KDL::JntArray upper_pos_limits_;
  KDL::JntArray lower_pos_limits_;
};

}  // namespace task_priority_controllers
