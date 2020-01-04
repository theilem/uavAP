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
 * LocalPlannerTargets.h
 *
 *  Created on: Aug 2, 2018
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_LOCALPLANNER_LOCALPLANNERTARGETS_H_
#define UAVAP_FLIGHTCONTROL_LOCALPLANNER_LOCALPLANNERTARGETS_H_

#include <cpsCore/Utilities/EnumMap.hpp>

enum class LocalPlannerTargets
{
	INVALID = 0,
	POSITION_X,
	POSITION_Y,
	POSITION_Z,
	DIRECTION_X,
	DIRECTION_Y,
	DIRECTION_Z,
	VELOCITY,
	HEADING,
	CLIMB_RATE,
	SLOPE,
	NUM_TARGETS
};

ENUMMAP_INIT(LocalPlannerTargets, {
		{LocalPlannerTargets::POSITION_X, "position_x"},
 		{LocalPlannerTargets::POSITION_Y, "position_y"},
		{LocalPlannerTargets::POSITION_Z, "position_z"},
		{LocalPlannerTargets::DIRECTION_X, "direction_x"},
		{LocalPlannerTargets::DIRECTION_Y, "direction_y"},
		{LocalPlannerTargets::DIRECTION_Z, "direction_z"},
		{LocalPlannerTargets::VELOCITY, "velocity"},
		{LocalPlannerTargets::HEADING, "heading"},
		{LocalPlannerTargets::CLIMB_RATE, "climb_rate"},
		{LocalPlannerTargets::SLOPE, "slope"}
});

#endif /* UAVAP_FLIGHTCONTROL_LOCALPLANNER_LOCALPLANNERTARGETS_H_ */
