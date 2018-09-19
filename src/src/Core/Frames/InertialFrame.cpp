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
 * InertialFrame.cpp
 *
 *  Created on: Aug 8, 2018
 *      Author: mircot
 */
#include <uavAP/Core/Frames/InertialFrame.h>




Vector3
InertialFrame::toInertialFramePosition(const Vector3& pos) const
{
	return pos;
}

Vector3
InertialFrame::toInertialFrameDirection(const Vector3& dir) const
{
	return dir;
}

Vector3
InertialFrame::toInertialFrameRotation(const Vector3& rot) const
{
	return rot;
}

Vector3
InertialFrame::fromFramePosition(const IFrame& orig, const Vector3& pos) const
{
	return orig.toInertialFramePosition(pos);
}

Vector3
InertialFrame::fromFrameDirection(const IFrame& orig, const Vector3& dir) const
{
	return orig.toInertialFrameDirection(dir);
}

Vector3
InertialFrame::fromFrameRotation(const IFrame& orig, const Vector3& rot) const
{
	return orig.toInertialFrameRotation(rot);
}

Frame
InertialFrame::getId() const
{
	return Frame::INERTIAL;
}
