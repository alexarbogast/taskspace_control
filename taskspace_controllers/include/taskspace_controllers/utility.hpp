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

#include <Eigen/Dense>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarrayvel.hpp>

namespace ctrl
{

typedef Eigen::Matrix<double, 6, 1> Vector6D;
typedef Eigen::Matrix<double, 5, 1> Vector5D;
typedef Eigen::Matrix<double, 4, 1> Vector4D;
typedef Eigen::Vector3d Vector3D;
typedef Eigen::Vector2d Vector2D;
typedef Eigen::VectorXd VectorND;
typedef Eigen::Matrix3d Matrix3D;
typedef Eigen::Matrix<double, 6, 6> Matrix6D;
typedef Eigen::MatrixXd MatrixND;
typedef Eigen::Quaterniond Quaternion;
typedef Eigen::AngleAxisd AngleAxis;
typedef Eigen::Isometry3d Pose;

/**
 * @brief Find the left pseudoinverse of a matrix
 *
 * Returns the left Moore-Penrose pseudoinverse of a "tall" (more rows than
 * columns) matrix with linearly independent columns.
 *
 * @param matrix the matrix on which to perform the pseudoinverse
 * @returns the left pseudoinverse of "matrix"
 */
MatrixND leftPinv(const MatrixND& matrix);

/**
 * @brief Find the right pseudoinverse of a matrix
 *
 * Returns the right Moore-Penrose pseudoinverse of a "wide" (more columns than
 * rows) matrix with linearly independent rows.
 *
 * @param matrix the matrix on which to perform the pseudoinverse
 * @returns the right pseudoinverse of "matrix"
 */
MatrixND rightPinv(const MatrixND& matrix);

/**
 * @brief Find the damped pseudoinverse of a matrix
 *
 * @param matrix the matrix on which to perform the pseudoinverse
 * @param alpha the damping factor between 0 and 1
 * @returns the damped pseudoinverse of "matrix"
 */
MatrixND dampedPinv(const MatrixND& matrix, double alpha);

/**
 * @brief Transforms a KDL frame to an Eigen::Isometry3d
 *
 * @param k the KDL frame to transform
 * @param e the eigen element to populate with the data from k
 */
void transformKDLToEigen(const KDL::Frame& k, Eigen::Isometry3d& e);

/**
 * @brief Transforms a KDL Rotation to an Eigen::Matrix
 *
 * @param k the KDL rotation to transform
 * @param e the eigen element to populate with the data from k
 */
void transformKDLToEigen(const KDL::Rotation& k,
                         Eigen::Matrix<double, 3, 3>& e);

/**
 * @brief Transforms a robot state (position + velocity) to a KDL::JntArrayVel
 *
 * @param q the joint position
 * @param qdot the joint velocity
 */
KDL::JntArrayVel transformEigenToKDL(const ctrl::VectorND& q,
                                     const ctrl::VectorND& qdot);

/**
 * @brief Determine if a list of interfaces includes a certain type
 *
 * @param interface_type_list a list of possible interfaces
 * @param interface_type the type to find in the list of interfaces
 */
bool contains_interface_type(
    const std::vector<std::string>& interface_type_list,
    const std::string& interface_type);

/**
 * @brief Returns a list of joints found along the kinematic chain
 *
 * @param interface_type_list a list of posible interfaces
 * @param interface_type the type to find in the list of interfaces
 */
std::vector<std::string> joints_along_chain(const KDL::Chain& chain,
                                            bool include_fixed = false);

}  // namespace ctrl
