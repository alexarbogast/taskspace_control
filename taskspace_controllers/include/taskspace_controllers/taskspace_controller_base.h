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

#include <taskspace_controllers/utility.h>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <taskspace_control_msgs/QueryPose.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace taskspace_controllers
{

class TaskspaceControllerBase : public controller_interface::Controller<
                                    hardware_interface::PositionJointInterface>
{
public:
  virtual bool init(hardware_interface::PositionJointInterface* hw,
                    ros::NodeHandle& nh) override;

protected:
  void synchronizeJointStates();
  void writeCommand(const ctrl::VectorND cmd);

  unsigned int n_joints_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::string base_link_, eef_link_;
  std::string setpoint_topic_;

  // kinematics
  KDL::Chain robot_chain_;
  KDL::JntArray upper_pos_limits_;
  KDL::JntArray lower_pos_limits_;

  // kinematic
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> robot_fk_solver_;

  // state feedback
  KDL::JntArrayVel robot_state_;

private:
  bool queryPoseService(taskspace_control_msgs::QueryPose::Request& req,
                        taskspace_control_msgs::QueryPose::Response& resp);

  ros::ServiceServer query_pose_service_;
  bool initialized_ = false;
};

}  // namespace taskspace_controllers
