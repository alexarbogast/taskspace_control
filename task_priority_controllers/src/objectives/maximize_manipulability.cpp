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

#include "task_priority_controllers/objectives/maximize_manipulability.hpp"
#include "taskspace_controllers/utility.hpp"

namespace task_priority_controllers
{

static double MANIP_THRESHOLD = 1e-10;

bool MaximizeManipulability::init(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const KDL::Chain& chain, const std::vector<ctrl::JointLimits>& joint_limits)
{
  if (!RRObjective::init(node, chain, joint_limits))
  {
    return false;
  }

  robot_jacobian_solver_ =
      std::make_unique<KDL::ChainJntToJacSolver>(robot_chain_);

  robot_jacobian_dot_solver_ =
      std::make_unique<KDL::ChainJntToJacDotSolver>(robot_chain_);

  try
  {
    param_listener_ =
        std::make_shared<maximize_manipulability::ParamListener>(node);
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
MaximizeManipulability::getJointControlCmd(const KDL::JntArrayVel& joint_state)
{
  params_ = param_listener_->get_params();

  KDL::Jacobian jac(n_joints_);
  robot_jacobian_solver_->JntToJac(joint_state.q, jac);

  ctrl::MatrixND J_JT = jac.data * jac.data.transpose();
  double manip = sqrt(J_JT.determinant());

  ctrl::VectorND manip_grad = ctrl::VectorND::Zero(n_joints_);
  if (manip < MANIP_THRESHOLD)
  {
    return manip_grad;
  }

  ctrl::MatrixND J_JT_inv = J_JT.inverse();
  KDL::JntArrayVel current_state(joint_state);
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
  return manip_grad *= manip * params_.k_manip;
}

}  // namespace task_priority_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(task_priority_controllers::MaximizeManipulability,
                       task_priority_controllers::RRObjective)
