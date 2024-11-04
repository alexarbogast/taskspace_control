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
#include <taskspace_controllers/setpoint.h>
#include <taskspace_controllers/PoseControllerConfig.h>

#include <kdl/chainjnttojacsolver.hpp>

#include <taskspace_control_msgs/PoseTwistSetpoint.h>
#include <realtime_tools/realtime_buffer.h>
#include <dynamic_reconfigure/server.h>

namespace taskspace_controllers
{

class PoseController : public TaskspaceControllerBase
{
public:
  virtual bool init(hardware_interface::PositionJointInterface* hw,
                    ros::NodeHandle& nh) override;
  virtual void update(const ros::Time&, const ros::Duration& period) override;
  virtual void starting(const ros::Time&) override;
  virtual void stopping(const ros::Time&) override;

protected:
  typedef TaskspaceControllerBase Base;
  typedef PoseTwistSetpoint Setpoint;
  typedef PoseControllerConfig ControllerConfig;
  typedef dynamic_reconfigure::Server<PoseControllerConfig> ReconfigureServer;

  void reconfCallback(ControllerConfig& config, uint16_t /*level*/);
  void setpointCallback(
      const taskspace_control_msgs::PoseTwistSetpointConstPtr& msg);

  // kinematics solvers
  std::unique_ptr<KDL::ChainJntToJacSolver> robot_jacobian_solver_;

  // setpoint
  realtime_tools::RealtimeBuffer<Setpoint> setpoint_;
  ros::Subscriber sub_setpoint_;

  // dynamic reconfigure
  struct DynamicParams
  {
    DynamicParams() : k_position(1.0), k_orient(1.0) {}

    double k_position;  // gain values
    double k_orient;
  };
  realtime_tools::RealtimeBuffer<DynamicParams> dynamic_params_;
  std::shared_ptr<ReconfigureServer> dyn_reconf_server_;
};

}  // namespace taskspace_controllers
