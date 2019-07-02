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
/*
 * LinearAlgebra.cpp
 *
 *  Created on: Jun 26, 2017
 *      Author: mircot
 */

#include <iostream>

#include "uavAP/Core/LinearAlgebra.h"

Vector2
rotate2Drad(const Vector2& vec, const FloatingType& rad)
{
	FloatingType c = std::cos(rad);
	FloatingType s = std::sin(rad);
	FloatingType x = vec.x() * c - vec.y() * s;
	FloatingType y = vec.x() * s + vec.y() * c;
	return Vector2(x, y);
}

std::istream&
operator>>(std::istream& is, Vector3& obj)
{
	is >> obj[0];
	is >> obj[1];
	is >> obj[2];
	return is;
}

std::ostream&
operator<<(std::ostream& os, const Vector3& obj)
{
	os << obj[0] << " " << obj[1] << " " << obj[2] << " ";
	return os;
}

std::istream&
operator>>(std::istream& is, EigenLine& obj)
{
	Vector3 origin, direction;
	is >> origin;
	is >> direction;
	obj = EigenLine(origin, direction);
	return is;
}

std::ostream&
operator<<(std::ostream& os, const EigenLine& obj)
{
	os << obj.origin() << obj.direction();
	return os;
}

std::istream&
operator>>(std::istream& is, EigenHyperplane& obj)
{
	Vector3 norm, off;
	is >> norm >> off;
	obj.normal();
	obj.offset();
	obj = EigenHyperplane(norm, off);
	return is;
}

Vector3&
degToRadRef(Vector3& vec)
{
	vec = vec * M_PI / 180.0;
	return vec;
}

FloatingType&
degToRadRef(FloatingType& deg)
{
	deg = deg * M_PI / 180.0;
	return deg;
}

Vector3&
radToDegRef(Vector3& vec)
{
	vec = vec * 180.0 / M_PI;
	return vec;
}

FloatingType&
radToDegRef(FloatingType& rad)
{
	rad = rad * 180.0 / M_PI;
	return rad;
}

Vector3
degToRad(const Vector3& vec)
{
	return vec * M_PI / 180.0;
}

FloatingType
degToRad(const FloatingType& deg)
{
	return deg * M_PI / 180.0;
}

Vector3
radToDeg(const Vector3& vec)
{
	return vec * 180.0 / M_PI;
}

FloatingType
radToDeg(const FloatingType& rad)
{
	return rad * 180.0 / M_PI;
}

std::ostream&
operator<<(std::ostream& os, const EigenHyperplane& obj)
{
	os << obj.normal() << obj.offset();
	return os;
}

FloatingType
headingFromENU(const Vector3& vec)
{
	return atan2(vec.y(), vec.x());
}

FloatingType
headingFromENU(const Vector2& vec)
{
	return atan2(vec.y(), vec.x());
}

FloatingType
boundAngleRad(FloatingType angle)
{
	if (angle > M_PI)
		angle -= 2 * M_PI;
	else if (angle < -M_PI)
		angle += 2 * M_PI;
	return angle;
}

Eigen::Quaternion<FloatingType>
eulerToQuaternion(const Vector3& euler)
{
	FloatingType pitchForQuat = boundAngleRad(euler[1] - M_PI);
	FloatingType yawForQuat = boundAngleRad(-euler[2] - M_PI / 2.0);
	Eigen::Quaternion<FloatingType> attitude(
			AngleAxis(yawForQuat, Vector3::UnitZ())
					* AngleAxis(pitchForQuat, Vector3::UnitY())
					* AngleAxis(euler[0], Vector3::UnitX()));
	attitude.normalize();
	return attitude;
}

Vector3
quaternionToEuler(const Eigen::Quaternion<FloatingType>& quaternion)
{
	Vector3 temp = quaternion.toRotationMatrix().eulerAngles(2, 1, 0);

	FloatingType roll = temp[2];
	FloatingType pitch = temp[1];
	FloatingType yaw = temp[0];
	bool flipPitch = (pitch > M_PI / 2) || (pitch < -M_PI / 2);

	if (flipPitch)
		pitch = boundAngleRad(-pitch - M_PI);
	else
	{
		yaw = boundAngleRad(yaw - M_PI);
		roll = boundAngleRad(roll - M_PI);
	}

	yaw = boundAngleRad(-(yaw - M_PI/2));

	return Vector3(roll, pitch, yaw);
}

Vector3
directionFromAttitude(const Vector3& att)
{
	return Vector3(cos(att[2]) * cos(att[1]), sin(att[2]) * cos(att[1]), sin(att[1])).normalized();
}
