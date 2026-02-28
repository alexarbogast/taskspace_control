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

#include "taskspace_controllers/utility.hpp"

namespace ctrl
{

MatrixND leftPinv(const MatrixND& matrix)
{
  return (matrix.transpose() * matrix).inverse() * matrix.transpose();
}

MatrixND rightPinv(const MatrixND& matrix)
{
  return matrix.transpose() * (matrix * matrix.transpose()).inverse();
}

MatrixND dampedPinv(const MatrixND& matrix, double alpha)
{
  MatrixND identity = MatrixND::Identity(matrix.rows(), matrix.rows());
  return matrix.transpose() *
         (matrix * matrix.transpose() + alpha * alpha * identity).inverse();
}

void transformKDLToEigen(const KDL::Frame& k, Eigen::Isometry3d& e)
{
  e = Pose::Identity();
  e.linear() =
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(k.M.data);
  e.translation() = Eigen::Map<const Eigen::Vector3d>(k.p.data);
}

void transformKDLToEigen(const KDL::Rotation& k, Eigen::Matrix<double, 3, 3>& e)
{
  e = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(k.data);
}

KDL::JntArrayVel transformEigenToKDL(const ctrl::VectorND& q,
                                     const ctrl::VectorND& qdot)
{
  const size_t n = q.size();
  KDL::JntArrayVel out(n);

  Eigen::Map<Eigen::VectorXd>(out.q.data.data(), n) = q;
  Eigen::Map<Eigen::VectorXd>(out.qdot.data.data(), n) = qdot;

  return out;
}

bool contains_interface_type(
    const std::vector<std::string>& interface_type_list,
    const std::string& interface_type)
{
  return std::find(interface_type_list.begin(), interface_type_list.end(),
                   interface_type) != interface_type_list.end();
}

std::vector<std::string> joints_along_chain(const KDL::Chain& chain,
                                            bool include_fixed)
{
  std::vector<std::string> joint_names;
  joint_names.reserve(chain.getNrOfJoints());

  for (unsigned int i = 0; i < chain.getNrOfSegments(); i++)
  {
    KDL::Joint joint = chain.getSegment(i).getJoint();

    if (include_fixed || joint.getType() != KDL::Joint::None)
    {
      joint_names.push_back(joint.getName());
    }
  }

  return joint_names;
}

}  // namespace ctrl
