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
	INVALID, ROLL, ROLL_RATE, PITCH, PITCH_RATE, NUM_CONSTRAINT
};

ENUMMAP_INIT(ControllerConstraints, { {ControllerConstraints::ROLL, "roll"},
		{ControllerConstraints::ROLL_RATE, "roll_rate"}, {ControllerConstraints::PITCH, "pitch"},
		{ControllerConstraints::PITCH_RATE, "pitch_rate"} });

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_CONTROLLERCONSTRAINT_H_ */
