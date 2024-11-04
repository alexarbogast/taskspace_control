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

#include <axially_symmetric_controllers/twist_decomposition_controller.h>
#include <axially_symmetric_controllers/utility.h>

#include <pluginlib/class_list_macros.h>

namespace axially_symmetric_controllers
{

static double MANIP_THRESHOLD = 1e-10;

static const std::string DIAGNOSTICS_NS = "diagnostics";

bool TwistDecompositionController::init(
    hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh)
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

  sub_setpoint_ =
      nh.subscribe(setpoint_topic_, 1,
                   &TwistDecompositionController::setpointCallback, this);

  // Dynamic reconfigure
  dyn_reconf_server_ = std::make_shared<ReconfigureServer>(nh);
  dyn_reconf_server_->setCallback(
      std::bind(&TwistDecompositionController::reconfCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  return true;
}

void TwistDecompositionController::update(const ros::Time&,
                                          const ros::Duration& period)
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

  ctrl::Vector3D orient_error = rot_angle * rot_axis;
  ctrl::Vector3D pos_error((setpoint->pose.p - pose.p).data);

  ctrl::Vector6D cart_cmd;
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
  ctrl::VectorND h = manip_grad;

  /* task decomposition */
  ctrl::Vector3D e(pose.M.UnitZ().data);
  ctrl::Matrix3D eeT = e * e.transpose();

  ctrl::Matrix6D T = Eigen::Matrix<double, 6, 6>::Zero();
  T.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  T.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() - eeT;

  ctrl::Vector3D perp_cmd = eeT * jac.data.block(3, 0, 3, n_joints_) * h;

  /* control */
  ctrl::MatrixND Jr = jac.data;
  ctrl::MatrixND Jr_pinv = ctrl::rightPinv(Jr);

  ctrl::Vector6D mod_cart_cmd = T * cart_cmd;
  mod_cart_cmd.block<3, 1>(3, 0) += perp_cmd;

  ctrl::VectorND joint_cmd = Jr_pinv * mod_cart_cmd;

  ctrl::VectorND new_position =
      robot_state_.q.data + (joint_cmd * period.toSec());

  writeCommand(new_position);
}

void TwistDecompositionController::starting(const ros::Time&)
{
  synchronizeJointStates();

  Setpoint init_setpoint;
  robot_fk_solver_->JntToCart(robot_state_.q, init_setpoint.pose);
  setpoint_.initRT(init_setpoint);
}

void TwistDecompositionController::stopping(const ros::Time&) {}

void TwistDecompositionController::reconfCallback(ControllerConfig& config,
                                                  uint16_t /*level*/)
{
  DynamicParams dynamic_params;
  dynamic_params.k_position = config.k_position;
  dynamic_params.k_aiming = config.k_aiming;
  dynamic_params.k_manip = config.k_manip;
  dynamic_params.k_limits = config.k_limits;

  dynamic_params_.writeFromNonRT(dynamic_params);
}

void TwistDecompositionController::setpointCallback(
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

PLUGINLIB_EXPORT_CLASS(
    axially_symmetric_controllers::TwistDecompositionController,
    controller_interface::ControllerBase)
