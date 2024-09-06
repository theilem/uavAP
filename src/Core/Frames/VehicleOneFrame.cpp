/*
 * VehicleOneFrame.cpp
 *
 *  Created on: Aug 8, 2018
 *      Author: mircot
 */
#include <uavAP/Core/Frames/VehicleOneFrame.h>

VehicleOneFrame::VehicleOneFrame() :
		yaw_(0), origin_(0,0,0)
{
}

VehicleOneFrame::VehicleOneFrame(FloatingType yaw, const Vector3& origin) :
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
	return AngleAxis(yaw_, Vector3::UnitZ()) * dir;
}

Vector3
VehicleOneFrame::toInertialFrameRotation(const Vector3& rot) const
{
	Vector3 res = rot + Vector3(0, 0, yaw_);
	res[2] = boundAngleRad(res[2]);
	return res;
}

FloatingType
VehicleOneFrame::toInertialFrameCourse(FloatingType chi) const
{
	return chi + yaw_;
}

Vector3
VehicleOneFrame::fromFramePosition(const IFrame& orig, const Vector3& pos) const
{
	return AngleAxis(-yaw_, Vector3::UnitZ())
			* (orig.toInertialFramePosition(pos) - origin_);
}

Vector3
VehicleOneFrame::fromFrameDirection(const IFrame& orig, const Vector3& dir) const
{
	return AngleAxis(-yaw_, Vector3::UnitZ()) * (orig.toInertialFrameDirection(dir));
}

Vector3
VehicleOneFrame::fromFrameRotation(const IFrame& orig, const Vector3& rot) const
{
	Vector3 res = orig.toInertialFrameRotation(rot) - Vector3(0, 0, yaw_);
	res[2] = boundAngleRad(res[2]);
	return res;
}

FloatingType
VehicleOneFrame::fromFrameCourse(const IFrame &orig, FloatingType chi) const
{
	return orig.toInertialFrameCourse(chi) - yaw_;;
}

Frame
VehicleOneFrame::getId() const
{
	return Frame::VEHICLE_1;
}

void
VehicleOneFrame::setYaw(FloatingType yaw)
{
	yaw_ = yaw;
}
void
VehicleOneFrame::setOrigin(const Vector3& origin)
{
	origin_ = origin;
}

FloatingType
VehicleOneFrame::getYaw() const
{
	return yaw_;
}

const Vector3&
VehicleOneFrame::getOrigin() const
{
	return origin_;
}
