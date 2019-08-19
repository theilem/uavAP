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
 * PIDMapping.h
 *
 *  Created on: Jul 23, 2018
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_PIDMAPPING_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_PIDMAPPING_H_

#include <string>
#include "uavAP/Core/EnumMap.hpp"

enum class PIDs
{
	INVALID = 0,
	VELOCITY,
	VELOCITY_X,
	VELOCITY_Y,
	RPM,
	RUDDER,
	CLIMB_ANGLE,
	CLIMB_RATE,
	PITCH,
	PITCH_RATE,
	YAW_RATE,
	ROLL,
	ROLL_RATE,
	NUM_PID
};

ENUMMAP_INIT(PIDs,
		{{PIDs::INVALID, "invalid"},
		{PIDs::VELOCITY, "velocity"},
		{PIDs::VELOCITY_X, "velocity_x"},
		{PIDs::VELOCITY_Y, "velocity_y"},
		{PIDs::RPM, "rpm"},
		{PIDs::RUDDER, "rudder"},
		{PIDs::CLIMB_ANGLE,	"climb_angle"},
		{PIDs::CLIMB_RATE,	"climb_rate"},
		{PIDs::PITCH, "pitch"},
		{PIDs::PITCH_RATE, "pitch_rate"},
		{PIDs::YAW_RATE, "yaw_rate"},
		{PIDs::ROLL, "roll"},
		{PIDs::ROLL_RATE, "roll_rate"}});

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_PIDMAPPING_H_ */
