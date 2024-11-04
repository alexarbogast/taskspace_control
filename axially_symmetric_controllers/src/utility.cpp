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

#include <axially_symmetric_controllers/utility.h>

namespace axially_symmetric_controllers
{

double angleBetween(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
  return acos(
      std::min(1.0, std::max(-1.0, v1.dot(v2) / (v1.norm() * v2.norm()))));
}

Eigen::Vector3d axisBetween(const Eigen::Vector3d& v1,
                            const Eigen::Vector3d& v2, double tol)
{
  Eigen::Vector3d axis = v1.cross(v2);
  double norm = axis.norm();
  if (norm > tol)
  {
    return axis / norm;
  }
  return Eigen::Vector3d::Zero();
}

}  // namespace axially_symmetric_controllers
