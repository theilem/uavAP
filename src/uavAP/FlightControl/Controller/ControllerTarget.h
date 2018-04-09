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
 *  @file         ControllerTarget.h
 *  @author Mirco Theile
 *  @date      23 June 2017
 *  @brief      UAV Autopilot Controller Target Header File
 *
 *  Description
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_CONTROLLERTARGET_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_CONTROLLERTARGET_H_
#include "uavAP/Core/LinearAlgebra.h"

struct ControllerTarget
{
	Vector3 velocity; //Vehicle-1-Frame: Only dependend on heading -> x,y,Z
	double yawRate;
	double climbAngle;

	uint32_t sequenceNr; //Trace sequence number to get timing

	ControllerTarget() :
			velocity(0, 0, 0), yawRate(0), climbAngle(0), sequenceNr(0)
	{
	}
};

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_CONTROLLERTARGET_H_ */
