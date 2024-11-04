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
#include <axially_symmetric_controllers/setpoint.h>
#include <axially_symmetric_controllers/AxiallySymmetricControllerConfig.h>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>

#include <taskspace_control_msgs/PoseTwistSetpoint.h>
#include <realtime_tools/realtime_buffer.h>
#include <dynamic_reconfigure/server.h>

namespace axially_symmetric_controllers
{

class TwistDecompositionController
  : public taskspace_controllers::TaskspaceControllerBase
{
public:
  virtual bool init(hardware_interface::PositionJointInterface* hw,
                    ros::NodeHandle& nh) override;
  virtual void update(const ros::Time&, const ros::Duration& period) override;
  virtual void starting(const ros::Time&) override;
  virtual void stopping(const ros::Time&) override;

protected:
  typedef taskspace_controllers::TaskspaceControllerBase Base;
  typedef AxiallySymmetricSetpoint Setpoint;
  typedef AxiallySymmetricControllerConfig ControllerConfig;
  typedef dynamic_reconfigure::Server<AxiallySymmetricControllerConfig>
      ReconfigureServer;

  void reconfCallback(ControllerConfig& config, uint16_t /*level*/);
  void setpointCallback(
      const taskspace_control_msgs::PoseTwistSetpointConstPtr& msg);

  // kinematic solvers
  std::unique_ptr<KDL::ChainJntToJacSolver> robot_jacobian_solver_;
  std::unique_ptr<KDL::ChainJntToJacDotSolver> robot_jacobian_dot_solver_;

  // limits
  KDL::JntArray limits_avg_;
  KDL::JntArray limits_bounds_;

  // setpoint
  realtime_tools::RealtimeBuffer<Setpoint> setpoint_;
  ros::Subscriber sub_setpoint_;

  // dynamic reconfigure
  struct DynamicParams
  {
    DynamicParams()
      : k_position(1.0), k_aiming(1.0), k_manip(1.0), k_limits(1.0)
    {
    }

    double k_position;  // gain values
    double k_aiming;
    double k_manip;
    double k_limits;
  };
  realtime_tools::RealtimeBuffer<DynamicParams> dynamic_params_;
  std::shared_ptr<ReconfigureServer> dyn_reconf_server_;
};

}  // namespace axially_symmetric_controllers
