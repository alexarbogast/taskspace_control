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

#include "task_priority_controllers/objectives/match_configuration.hpp"

namespace task_priority_controllers
{

const static std::string CONFIG_PARAM = "match_config";

bool MatchConfiguration::init(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const KDL::Chain& chain, const KDL::JntArray& upper_pos_limits,
    const KDL::JntArray& lower_pos_limits)
{
  if (!RRObjective::init(node, chain, upper_pos_limits, lower_pos_limits))
  {
    return false;
  }

  try
  {
    param_listener_ =
        std::make_shared<match_configuration::ParamListener>(node);
  }
  catch (const std::exception& e)
  {
    fprintf(stderr,
            "Exception thrown during rr objective init with message: %s \n",
            e.what());
    return false;
  }

  params_ = param_listener_->get_params();
  config_.data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      params_.match_config.data(), params_.match_config.size());

  if (params_.match_config.size() != n_joints_)
  {
    const auto msg = std::string("Number of joints in ") +
                     node->get_namespace() + "/" + CONFIG_PARAM +
                     " does not match robot chain";
    RCLCPP_ERROR(node->get_logger(), "%s", msg.c_str());
    return false;
  }

  return true;
}

ctrl::VectorND
MatchConfiguration::getJointControlCmd(const KDL::JntArrayVel& joint_state)
{
  // const DynamicParams* params = dynamic_params_.readFromRT();
  params_ = param_listener_->get_params();
  return params_.k_config * (config_.data - joint_state.q.data);
}

}  // namespace task_priority_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(task_priority_controllers::MatchConfiguration,
                       task_priority_controllers::RRObjective)
