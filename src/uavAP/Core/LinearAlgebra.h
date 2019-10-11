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
 *  @author  Mirco Theile, mirco.theile@tum.de
 *  @date      23 June 2017
 *  @brief      Vector handling, conversions and rotations
 */

#ifndef UAVAP_CORE_LINEARALGEBRA_H_
#define UAVAP_CORE_LINEARALGEBRA_H_

#define EIGEN_DONT_ALIGN_STATICALLY

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h"

#ifdef FLOAT_SINGLE
#error This is wrong

using FloatingType = float;
using Vector2 = Eigen::Matrix<float, 2, 1, Eigen::DontAlign>;
using Vector3 = Eigen::Matrix<float, 3, 1, Eigen::DontAlign>;
using Matrix3 = Eigen::Matrix<float, 3, 3, Eigen::DontAlign>;
using AngleAxis = Eigen::AngleAxisf;
using Rotation2 = Eigen::Rotation2Df;
using EigenLine = Eigen::ParametrizedLine<float, 3, Eigen::DontAlign>;
using EigenLine2 = Eigen::ParametrizedLine<float, 2, Eigen::DontAlign>;
using EigenHyperplane = Eigen::Hyperplane<float, 3, Eigen::DontAlign>;

#else

using FloatingType = double;
using Vector2 = Eigen::Vector2d;
using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;
using AngleAxis = Eigen::AngleAxisd;
using Rotation2 = Eigen::Rotation2Dd;
using EigenLine = Eigen::ParametrizedLine<double, 3>;
using EigenLine2 = Eigen::ParametrizedLine<double, 2>;
using EigenHyperplane = Eigen::Hyperplane<double, 3, Eigen::DontAlign>;

#endif

/**
 * @brief Rotate 2D vector counter clockwise
 * @param vec Vector to be rotated
 * @param rad Rotation angle in radians
 * @return Rotated vector
 */
Vector2
rotate2Drad(const Vector2& vec, const FloatingType& rad);

/**
 * @brief Caculate the Heading from a Vector3 in ENU
 * @param vec Vector3 in ENU
 * @return Heading in radians. North is pi/2, East is 0.
 */
FloatingType
headingFromENU(const Vector3& vec);

/**
 * @brief Caculate the Heading from a Vector2 in EN(U)
 * @param vec Vector2 in EN(U)
 * @return Heading in radians. North is pi/2, East is 0.
 */
FloatingType
headingFromENU(const Vector2& vec);

/**
 * @brief Get the angle in a range between -PI and PI
 * @param angle Angle in radians
 * @return angle in (-PI, PI]
 */
FloatingType
boundAngleRad(FloatingType angle);

/**
 * @brief Convert euler angles to quaternion
 * @param euler Vector with [roll, pitch, yaw]
 * @return Quaternion
 */
Eigen::Quaternion<FloatingType>
eulerToQuaternion(const Vector3& euler);

/**
 * @brief Convert quaternion to euler angles
 * @param quaternion
 * @return euler angles [roll, pitch, yaw]
 */
Vector3
quaternionToEuler(const Eigen::Quaternion<FloatingType>& quaternion);

Vector3
directionFromAttitude(const Vector3& att);

Vector3&
degToRadRef(Vector3& vec);

FloatingType&
degToRadRef(FloatingType& deg);

Vector3&
radToDegRef(Vector3& vec);

FloatingType&
radToDegRef(FloatingType& rad);

Vector3
degToRad(const Vector3& vec);

FloatingType
degToRad(const FloatingType& deg);

Vector3
radToDeg(const Vector3& vec);

FloatingType
radToDeg(const FloatingType& rad);

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

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, Eigen::Vector3d& t)
{
	ar & t[0];
	ar & t[1];
	ar & t[2];
}

template<class Archive, typename Type>
inline void
serialize(Archive& ar, Vector2& t)
{
	ar & t[0];
	ar & t[1];
}

template<class Archive, typename Type>
inline void
serialize(Archive& ar, Eigen::Vector3f& t)
{
	ar & t[0];
	ar & t[1];
	ar & t[2];
}

template<class Archive, typename Type>
void
store(Archive& ar, const EigenLine& t)
{
	ar << t.origin();
	ar << t.direction();
}

template<class Archive, typename Type>
void
load(Archive& ar, EigenLine& t)
{
	Vector3 origin;
	Vector3 direction;
	ar >> origin;
	ar >> direction;
	t = EigenLine(origin, direction);
}

template<class Archive, typename Type>
inline void
serialize(Archive& ar, EigenLine& t)
{
	split(ar, t);
}
}

#endif /* UAVAP_CORE_LINEARALGEBRA_H_ */
