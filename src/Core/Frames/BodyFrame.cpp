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
 * BodyFrame.cpp
 *
 *  Created on: Aug 8, 2018
 *      Author: mircot
 */
#include <uavAP/Core/Frames/BodyFrame.h>

BodyFrame::BodyFrame(const Vector3& rotation, const Vector3& origin) :
		origin_(origin), rotation_(rotation), invInitialized_(false)
{
	// Fix this Eigen align issue
	rotationMatrix_ = AngleAxis(-rotation[0], Vector3::UnitX())
			* AngleAxis(-rotation[1], Vector3::UnitY())
			* AngleAxis(-rotation[2], Vector3::UnitZ());
}

Vector3
BodyFrame::toInertialFramePosition(const Vector3& pos) const
{
	return toInertialFrameDirection(pos) + origin_;
}

Vector3
BodyFrame::toInertialFrameDirection(const Vector3& dir) const
{
	initInvert();
	return rotationMatrixInv_ * dir;
}

Vector3
BodyFrame::toInertialFrameRotation(const Vector3& rot) const
{
	return rot - rotation_;
}

FloatingType
BodyFrame::toInertialFrameCourse(FloatingType chi) const
{
	return chi - rotation_[2];
}

Vector3
BodyFrame::fromFramePosition(const IFrame& orig, const Vector3& pos) const
{
	return rotationMatrix_ * (orig.toInertialFramePosition(pos) - origin_);
}

Vector3
BodyFrame::fromFrameDirection(const IFrame& orig, const Vector3& dir) const
{
	return rotationMatrix_ * orig.toInertialFrameDirection(dir);
}

Vector3
BodyFrame::fromFrameRotation(const IFrame& orig, const Vector3& rot) const
{
	return rotation_ + orig.toInertialFrameRotation(rot);
}

FloatingType
BodyFrame::fromFrameCourse(const IFrame &orig, FloatingType chi) const
{
	return rotation_[2] + orig.toInertialFrameCourse(chi);
}

Frame
BodyFrame::getId() const
{
	return Frame::BODY;
}

void
BodyFrame::initInvert() const
{
	if (invInitialized_)
		return;
	invInitialized_ = true;
	rotationMatrixInv_ = rotationMatrix_.inverse();
}
