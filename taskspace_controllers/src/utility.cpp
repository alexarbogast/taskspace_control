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

MatrixND pseudoInverse(const MatrixND& matrix)
{
  Eigen::CompleteOrthogonalDecomposition<MatrixND> cod(matrix);
  return cod.pseudoInverse();
}

MatrixND dampedPseudoInverse(const MatrixND& matrix, double alpha)
{
  const MatrixND identity = MatrixND::Identity(matrix.rows(), matrix.rows());
  const MatrixND A = matrix * matrix.transpose() + alpha * alpha * identity;

  return matrix.transpose() * A.ldlt().solve(identity);
}

ctrl::Matrix3D skew(const ctrl::Vector3D& v)
{
  Matrix3D S;
  // clang-format off
  S << 0.0,  -v.z(), v.y(),
       v.z(), 0.0,  -v.x(),
      -v.y(), v.x(), 0.0;
  // clang-format on
  return S;
}

double compute_manipulability(const KDL::Jacobian& jac)
{
  const auto& J = jac.data;
  const MatrixND J_JT = J * J.transpose();
  return std::sqrt(std::max(0.0, J_JT.determinant()));
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

KDL::JntArray transformEigenToKDL(const ctrl::VectorND& q)
{
  const size_t n = q.size();
  KDL::JntArray out(n);

  Eigen::Map<Eigen::VectorXd>(out.data.data(), n) = q;

  return out;
}

KDL::Rotation transformEigenToKDL(const Eigen::Matrix3d& R)
{
  // clang-format off
  return KDL::Rotation(
      R(0,0), R(0,1), R(0,2),
      R(1,0), R(1,1), R(1,2),
      R(2,0), R(2,1), R(2,2)
  );
  // clang-format on
}

geometry_msgs::msg::Pose transformEigenToROS(const ctrl::Pose& p)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = p.translation().x();
  msg.position.y = p.translation().y();
  msg.position.z = p.translation().z();

  const Quaternion q(p.rotation());
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();

  return msg;
}

geometry_msgs::msg::Vector3 transformEigenToROS(const ctrl::Vector3D& v)
{
  geometry_msgs::msg::Vector3 msg;

  msg.x = v.x();
  msg.y = v.y();
  msg.z = v.z();
  return msg;
}

void computePoseError(const ctrl::Pose& target, const ctrl::Pose& current,
                      ctrl::Vector3D& translation_error,
                      ctrl::Vector3D& orientation_error)
{
  translation_error = target.translation() - current.translation();

  const AngleAxis aa(target.rotation() * current.rotation().inverse());
  orientation_error = aa.axis() * aa.angle();
}

void integrate_joint_velocity(
    const ctrl::VectorND& initial_position,
    const ctrl::VectorND& joint_velocity_command,
    const std::vector<joint_limits::JointLimits>& joint_limits, double dt,
    KDL::JntArrayVel& command)
{
  const size_t n_joints = initial_position.size();
  for (size_t i = 0; i < n_joints; ++i)
  {
    const auto& limits = joint_limits[i];

    const double position = initial_position(i);
    double velocity = joint_velocity_command(i);

    // Velocity saturation
    if (!std::isnan(limits.max_velocity))
    {
      velocity =
          std::clamp(velocity, -limits.max_velocity, limits.max_velocity);
    }

    // Numerical Integration
    double next_position = position + velocity * dt;

    // Position saturation
    if (!std::isnan(limits.min_position) && !std::isnan(limits.max_position))
    {
      const double clamped_position =
          std::clamp(next_position, limits.min_position, limits.max_position);

      // Back-compute velocity to stay consistent
      velocity = (clamped_position - position) / dt;
      next_position = clamped_position;
    }

    command.q(i) = next_position;
    command.qdot(i) = velocity;
  }
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
