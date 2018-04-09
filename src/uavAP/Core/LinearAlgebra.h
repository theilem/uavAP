////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/**
 *  @file         LinearAlgebra.h
 *  @author  Mirco Theile, mircot@illinois.edu
 *  @date      23 June 2017
 *  @brief      Vector handling, conversions and rotations
 */

#ifndef UAVAP_CORE_LINEARALGEBRA_H_
#define UAVAP_CORE_LINEARALGEBRA_H_

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include <Eigen/Geometry>

using Vector2 = Eigen::Vector2d;
using Vector3 = Eigen::Vector3d;
using Rotation2 = Eigen::Rotation2Dd;
using EigenLine = Eigen::ParametrizedLine<double, 3>;
using EigenHyperplane = Eigen::Hyperplane<double, 3, Eigen::DontAlign>;

class PositionENU;

/**
 * @brief Convert protobuf ENUPosition into Eigen::Vector3d
 * @param position Position in ENU
 * @return Position as Eigen::Vector3d
 */
Vector3
toVector(const PositionENU& position);

/**
 * @brief Rotate 2D vector counter clockwise
 * @param vec Vector to be rotated
 * @param rad Rotation angle in radians
 * @return Rotated vector
 */
Vector2
rotate2Drad(Vector2 vec, double rad);

/**
 * @brief Caculate the Heading from a Vector3 in ENU
 * @param vec Vector3 in ENU
 * @return Heading in radians. North is 0, East is pi/2.
 */
double
headingFromENU(const Vector3& vec);

/**
 * @brief Caculate the Heading from a Vector2 in EN(U)
 * @param vec Vector2 in EN(U)
 * @return Heading in radians. North is 0, East is pi/2.
 */
double
headingFromENU(const Vector2& vec);

/**
 * @brief Get the angle in a range between -PI and PI
 * @param angle Angle in radians
 * @return angle in (-PI, PI]
 */
double
boundAngleRad(double angle);

std::istream&
operator>>(std::istream& is, Vector3& obj);
std::ostream&
operator<<(std::ostream& os, const Vector3& obj);
std::istream&
operator>>(std::istream& is, EigenLine& obj);
std::ostream&
operator<<(std::ostream& os, const EigenLine& obj);
std::istream&
operator>>(std::istream& is, EigenHyperplane& obj);
std::ostream&
operator<<(std::ostream& os, const EigenHyperplane& obj);

#endif /* UAVAP_CORE_LINEARALGEBRA_H_ */
