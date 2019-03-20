/*
 * ControllerConstraint.h
 *
 *  Created on: Mar 19, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_CONTROLLERCONSTRAINT_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_CONTROLLERCONSTRAINT_H_

#include "uavAP/Core/EnumMap.hpp"

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

ENUMMAP_INIT(ControllerConstraints, { {ControllerConstraints::ROLL, "roll"},
		{ControllerConstraints::ROLL_RATE, "roll_rate"}, {ControllerConstraints::ROLL_OUTPUT,
		"roll_output"}, {ControllerConstraints::PITCH, "pitch"}, {ControllerConstraints::PITCH_RATE,
		"pitch_rate"}, {ControllerConstraints::PITCH_OUTPUT, "pitch_output"},
		{ControllerConstraints::YAW_OUTPUT, "yaw_output"}, {ControllerConstraints::THROTTLE_OUTPUT,
		"throttle_output"} });

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_CONTROLLERCONSTRAINT_H_ */
