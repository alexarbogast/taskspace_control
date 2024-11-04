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

#include <axially_symmetric_controllers/nullspace_controller.h>
#include <axially_symmetric_controllers/utility.h>

#include <pluginlib/class_list_macros.h>

namespace axially_symmetric_controllers
{

static const Eigen::Matrix<double, 5, 5> identity5x5 =
    Eigen::Matrix<double, 5, 5>::Identity();

static double MANIP_THRESHOLD = 1e-10;

bool NullspaceController::init(hardware_interface::PositionJointInterface* hw,
                               ros::NodeHandle& nh)
{
  Base::init(hw, nh);

  robot_jacobian_solver_ =
      std::make_unique<KDL::ChainJntToJacSolver>(robot_chain_);

  robot_jacobian_dot_solver_ =
      std::make_unique<KDL::ChainJntToJacDotSolver>(robot_chain_);

  limits_avg_.resize(n_joints_);
  limits_avg_.data = (upper_pos_limits_.data + lower_pos_limits_.data) / 2;

  limits_bounds_.resize(n_joints_);
  limits_bounds_.data = (upper_pos_limits_.data - lower_pos_limits_.data) / 2;

  sub_setpoint_ = nh.subscribe(setpoint_topic_, 1,
                               &NullspaceController::setpointCallback, this);

  // Dynamic reconfigure
  dyn_reconf_server_ = std::make_shared<ReconfigureServer>(nh);
  dyn_reconf_server_->setCallback(
      std::bind(&NullspaceController::reconfCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  return true;
}

void NullspaceController::update(const ros::Time&, const ros::Duration& period)
{
  synchronizeJointStates();  // update state

  const DynamicParams* params = dynamic_params_.readFromRT();
  const Setpoint* setpoint = setpoint_.readFromRT();

  KDL::Jacobian jac(n_joints_);
  robot_jacobian_solver_->JntToJac(robot_state_.q, jac);

  KDL::Frame pose;
  robot_fk_solver_->JntToCart(robot_state_.q, pose);

  /* error */
  ctrl::Vector3D aim_current(pose.M.UnitZ().data);
  ctrl::Vector3D aim_desired(setpoint->pose.M.UnitZ().data);

  ctrl::Vector3D rot_axis = axisBetween(aim_current, aim_desired);
  double rot_angle = angleBetween(aim_current, aim_desired);

  ctrl::Vector2D orient_error(rot_axis.x(), rot_axis.y());
  orient_error *= rot_angle;

  ctrl::Vector3D pos_error((setpoint->pose.p - pose.p).data);

  ctrl::Vector5D cart_cmd;
  cart_cmd << params->k_position * pos_error + setpoint->velocity,
      params->k_aiming * orient_error;

  /* redundancy resolution */
  // manipulability maximization
  ctrl::MatrixND J_JT = jac.data * jac.data.transpose();
  double manip = sqrt(J_JT.determinant());

  ctrl::VectorND manip_grad(n_joints_);
  if (manip > MANIP_THRESHOLD)
  {
    ctrl::MatrixND J_JT_inv = J_JT.inverse();
    KDL::JntArrayVel current_state(robot_state_);
    KDL::Jacobian hessian_block(n_joints_);

    for (std::size_t i = 0; i < n_joints_; ++i)
    {
      current_state.qdot.data.setZero();
      current_state.qdot(i) = 1.0;

      robot_jacobian_dot_solver_->JntToJacDot(current_state, hessian_block);
      manip_grad[i] = (jac.data * hessian_block.data.transpose())
                          .cwiseProduct(J_JT_inv)
                          .sum();
    }
    manip_grad *= manip * params->k_manip;
  }
  else
  {
    manip_grad.setZero();
  }
  ctrl::VectorND h = manip_grad;  // temp

  /* control */
  ctrl::MatrixND Jr = jac.data.block(0, 0, 5, n_joints_);
  ctrl::MatrixND Jr_pinv = ctrl::rightPinv(Jr);

  static ctrl::MatrixND I = ctrl::MatrixND::Identity(n_joints_, n_joints_);
  ctrl::VectorND joint_cmd = Jr_pinv * cart_cmd + ((I - Jr_pinv * Jr) * h);

  ctrl::VectorND new_position =
      robot_state_.q.data + (joint_cmd * period.toSec());

  writeCommand(new_position);
}

void NullspaceController::starting(const ros::Time&)
{
  synchronizeJointStates();

  Setpoint init_setpoint;
  robot_fk_solver_->JntToCart(robot_state_.q, init_setpoint.pose);
  setpoint_.initRT(init_setpoint);
}

void NullspaceController::stopping(const ros::Time&) {}

void NullspaceController::reconfCallback(ControllerConfig& config,
                                         uint16_t /*level*/)
{
  DynamicParams dynamic_params;
  dynamic_params.k_position = config.k_position;
  dynamic_params.k_aiming = config.k_aiming;
  dynamic_params.k_manip = config.k_manip;
  dynamic_params.k_limits = config.k_limits;

  dynamic_params_.writeFromNonRT(dynamic_params);
}

void NullspaceController::setpointCallback(
    const taskspace_control_msgs::PoseTwistSetpointConstPtr& msg)
{
  Setpoint setpoint;
  setpoint.pose.p = KDL::Vector(msg->pose.position.x, msg->pose.position.y,
                                msg->pose.position.z);

  setpoint.pose.M = KDL::Rotation::Quaternion(
      msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
      msg->pose.orientation.w);

  setpoint.velocity << msg->twist.linear.x, msg->twist.linear.y,
      msg->twist.linear.z;

  setpoint_.writeFromNonRT(setpoint);
}

}  // namespace axially_symmetric_controllers

PLUGINLIB_EXPORT_CLASS(axially_symmetric_controllers::NullspaceController,
                       controller_interface::ControllerBase)
