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

#include <taskspace_controllers/utility.h>

namespace axially_symmetric_controllers
{

/**
 * @brief Find the angle (in radians) from vectors v1 to v2
 *
 * No checks are performed on the magnitude of the input vectors.
 *
 * @param v1 the first vector
 * @param v1 the second vector
 * @returns the angle in radians between v1 and v2
 */
double angleBetween(const ctrl::Vector3D& v1, const ctrl::Vector3D& v2);

/**
 * @brief Find the unit magnitude axis of rotation to rotate vector v1 to v2
 *
 * Returns a zero vector if the norm of the cross product is less than tol (i.e.
 * vectors are close to collinear). No checks are performed on the magnitude of
 * the input vectors.
 *
 * @param v1 the first vector
 * @param v2 the second vector
 * @param tol the upper bound of the divisor for axis normalization
 * @returns the unit vector axis of rotation
 */
ctrl::Vector3D axisBetween(const ctrl::Vector3D& v1, const ctrl::Vector3D& v2,
                           double tol = 1e-5);

}  // namespace axially_symmetric_controllers
