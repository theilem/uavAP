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
 * VehicleTwoFrame.h
 *
 *  Created on: Aug 8, 2018
 *      Author: mircot
 */

#ifndef UAVAP_CORE_FRAMES_VEHICLETWOFRAME_H_
#define UAVAP_CORE_FRAMES_VEHICLETWOFRAME_H_
#include <uavAP/Core/Frames/IFrame.h>

class VehicleTwoFrame : public IFrame
{
public:


	Vector3
	toInertialFramePosition(const Vector3& pos) const override;

	Vector3
	toInertialFrameDirection(const Vector3& dir) const override;

	Vector3
	toInertialFrameRotation(const Vector3& rot) const override;

	Vector3
	fromFramePosition(const IFrame& orig, const Vector3& pos) const override;

	Vector3
	fromFrameDirection(const IFrame& orig, const Vector3& dir) const override;

	Vector3
	fromFrameRotation(const IFrame& orig, const Vector3& rot) const override;

	Frame
	getId() const override;
};




#endif /* UAVAP_CORE_FRAMES_VEHICLETWOFRAME_H_ */
