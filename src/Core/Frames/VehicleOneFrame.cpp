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
 * VehicleOneFrame.cpp
 *
 *  Created on: Aug 8, 2018
 *      Author: mircot
 */
#include <uavAP/Core/Frames/VehicleOneFrame.h>

VehicleOneFrame::VehicleOneFrame() :
		yaw_(0)
{
}

VehicleOneFrame::VehicleOneFrame(double yaw, const Vector3& origin) :
		yaw_(yaw), origin_(origin)
{
}

Vector3
VehicleOneFrame::toInertialFramePosition(const Vector3& pos) const
{
	return toInertialFrameDirection(pos) + origin_;
}

Vector3
VehicleOneFrame::toInertialFrameDirection(const Vector3& dir) const
{
	return Eigen::AngleAxisd(yaw_, Vector3::UnitZ()) * dir;
}

Vector3
VehicleOneFrame::toInertialFrameRotation(const Vector3& rot) const
{
	Vector3 res = rot + Vector3(0, 0, yaw_);
	res[2] = boundAngleRad(res[2]);
	return res;
}

Vector3
VehicleOneFrame::fromFramePosition(const IFrame& orig, const Vector3& pos) const
{
	return Eigen::AngleAxisd(-yaw_, Vector3::UnitZ())
			* (orig.toInertialFramePosition(pos) - origin_);
}

Vector3
VehicleOneFrame::fromFrameDirection(const IFrame& orig, const Vector3& dir) const
{
	return Eigen::AngleAxisd(-yaw_, Vector3::UnitZ()) * (orig.toInertialFrameDirection(dir));
}

Vector3
VehicleOneFrame::fromFrameRotation(const IFrame& orig, const Vector3& rot) const
{
	Vector3 res = orig.toInertialFrameRotation(rot) - Vector3(0, 0, yaw_);
	res[2] = boundAngleRad(res[2]);
	return res;
}

Frame
VehicleOneFrame::getId() const
{
	return Frame::VEHICLE_1;
}

void
VehicleOneFrame::setYaw(double yaw)
{
	yaw_ = yaw;
}
void
VehicleOneFrame::setOrigin(const Vector3& origin)
{
	origin_ = origin;
}

double
VehicleOneFrame::getYaw() const
{
	return yaw_;
}

const Vector3&
VehicleOneFrame::getOrigin() const
{
	return origin_;
}
