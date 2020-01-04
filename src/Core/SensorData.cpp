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
 * SensorData.cpp
 *
 *  Created on: Jan 23, 2018
 *      Author: mircot
 */
#include <uavAP/Core/Frames/IFrame.h>
#include "uavAP/Core/SensorData.h"

SensorData&
changeFrame(const IFrame& orig, const IFrame& dest, SensorData& data)
{
	data.position = dest.fromFramePosition(orig, data.position);
	data.velocity = dest.fromFrameDirection(orig, data.velocity);
	data.attitude = dest.fromFrameRotation(orig, data.attitude);
	return data; // TODO: Should acceleration be changed?
}
