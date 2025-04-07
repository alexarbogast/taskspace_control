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

#include <task_priority_controllers/objectives/match_configuration.h>

namespace task_priority_controllers
{

const static std::string CONFIG_PARAM = "match_config";

bool MatchConfiguration::init(ros::NodeHandle& nh, const KDL::Chain& chain,
                              const KDL::JntArray& upper_pos_limits,
                              const KDL::JntArray& lower_pos_limits)
{
  if (!RRObjective::init(nh, chain, upper_pos_limits, lower_pos_limits))
  {
    return false;
  }

  // Read home configuration from ros parameters
  ros::NodeHandle pnh(nh, "rr_objective");
  std::vector<double> home_config;
  if (!pnh.getParam(CONFIG_PARAM, home_config))
  {
    ROS_ERROR_STREAM("Failed to load " << pnh.getNamespace() << "/"
                                       << CONFIG_PARAM
                                       << " from parameter server");
    return false;
  }
  config_.data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      home_config.data(), home_config.size());

  if (home_config.size() != n_joints_)
  {
    ROS_ERROR_STREAM("Number of joints in " << pnh.getNamespace() << "/"
                                            << CONFIG_PARAM
                                            << " does not match robot chain");
    return false;
  }
  return true;

  // Dynamic reconfigure
  dyn_reconf_server_ =
      std::make_shared<ReconfigureServer>(ros::NodeHandle(nh, "rr_objective"));
  dyn_reconf_server_->setCallback(std::bind(&MatchConfiguration::reconfCallback,
                                            this, std::placeholders::_1,
                                            std::placeholders::_2));
  return true;
}

ctrl::VectorND
MatchConfiguration::getJointControlCmd(const KDL::JntArrayVel& joint_state)
{
  const DynamicParams* params = dynamic_params_.readFromRT();
  return params->k_config * (config_.data - joint_state.q.data);
}

void MatchConfiguration::reconfCallback(ObjectiveConfig& config,
                                        uint16_t /*level*/)
{
  DynamicParams dynamic_params;
  dynamic_params.k_config = config.k_config;
  dynamic_params_.writeFromNonRT(dynamic_params);
}

}  // namespace task_priority_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(task_priority_controllers::MatchConfiguration,
                       task_priority_controllers::RRObjective)
