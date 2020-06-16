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

#include <cpsCore/Utilities/LinearAlgebra.h>
#include <cpsCore/Utilities/EnumMap.hpp>
#include <cpsCore/Utilities/DataPresentation/detail/SerializeCustom.h>

struct ControllerTarget : SerializeCustom
{
	FloatingType velocity;
	FloatingType yawRate;
	FloatingType climbAngle;

//	uint32_t sequenceNr; //Trace sequence number to get timing

	ControllerTarget() :
			velocity(0), yawRate(0), climbAngle(0)
//			, sequenceNr(0)
	{
	}
};

enum class ControllerTargets
{
	INVALID = 0, VELOCITY, YAW_RATE, CLIMB_ANGLE, NUM_TARGET
};

ENUMMAP_INIT(ControllerTargets, {
	{ ControllerTargets::VELOCITY, "velocity" },
	{ ControllerTargets::YAW_RATE, "yaw_rate" },
	{ ControllerTargets::CLIMB_ANGLE, "climb_angle" },
});

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, ControllerTarget& t)
{
	ar & t.velocity;
	ar & t.yawRate;
	ar & t.climbAngle;
}

}
#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_CONTROLLERTARGET_H_ */
