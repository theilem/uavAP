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
 * ControllerConstraint.h
 *
 *  Created on: Mar 19, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_CONTROLLERCONSTRAINT_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_CONTROLLERCONSTRAINT_H_

#include <cpsCore/Utilities/EnumMap.hpp>

enum class ControllerConstraints
{
	INVALID,
	ROLL,
	ROLL_RATE,
	ROLL_OUTPUT,
	PITCH,
	PITCH_RATE,
	PITCH_OUTPUT,
	YAW_OUTPUT,
	THROTTLE_OUTPUT,
	NUM_CONSTRAINT
};

ENUMMAP_INIT(ControllerConstraints, {
	{ ControllerConstraints::ROLL, "roll" },
	{ ControllerConstraints::ROLL_RATE, "roll_rate" },
	{ ControllerConstraints::ROLL_OUTPUT, "roll_output" },
	{ ControllerConstraints::PITCH, "pitch" },
	{ ControllerConstraints::PITCH_RATE, "pitch_rate" },
	{ ControllerConstraints::PITCH_OUTPUT, "pitch_output" },
	{ ControllerConstraints::YAW_OUTPUT, "yaw_output" },
	{ ControllerConstraints::THROTTLE_OUTPUT, "throttle_output" }
});

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_CONTROLLERCONSTRAINT_H_ */
