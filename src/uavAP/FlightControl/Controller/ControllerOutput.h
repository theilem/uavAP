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
 *  @file         ControllerOutput.h
 *  @author Simon Yu, Mirco Theile
 *  @date      26 June 2017
 *  @brief      UAV Autopilot Controller Output Header File
 *
 *  Description
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_CONTROLLEROUTPUT_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_CONTROLLEROUTPUT_H_

#include "uavAP/Core/DataPresentation/APDataPresentation/SerializeCustom.h"
#include "uavAP/Core/LinearAlgebra.h"

struct ControllerOutput: SerializeCustom
{
	double rollOutput;
	double pitchOutput;
	double yawOutput;
	double collectiveOutput;
	double throttleOutput;
	double flapOutput; //-1: no flap, 1: full flap

	uint32_t sequenceNr; //Trace sequence number to get timing

	ControllerOutput() :
			rollOutput(0), pitchOutput(0), yawOutput(0), collectiveOutput(0), throttleOutput(0), flapOutput(
					-1), sequenceNr(0)
	{
	}
};

struct ControllerOutputLight: SerializeCustom
{
	float rollOutput;
	float pitchOutput;
	float yawOutput;
	float collectiveOutput;
	float throttleOutput;
	float flapOutput; //-1: no flap, 1: full flap

	uint32_t sequenceNr; //Trace sequence number to get timing

	ControllerOutputLight() :
			rollOutput(0), pitchOutput(0), yawOutput(0), collectiveOutput(0), throttleOutput(0), flapOutput(
					-1), sequenceNr(0)
	{
	}
};

inline ControllerOutput
fromControllerOutputLight(const ControllerOutputLight& out)
{
	ControllerOutput con;
	con.rollOutput = static_cast<double>(out.rollOutput);
	con.pitchOutput = static_cast<double>(out.pitchOutput);
	con.yawOutput = static_cast<double>(out.yawOutput);
	con.collectiveOutput = static_cast<double>(out.collectiveOutput);
	con.throttleOutput = static_cast<double>(out.throttleOutput);
	con.flapOutput = static_cast<double>(out.flapOutput);
	con.sequenceNr = out.sequenceNr;
	return con;
}

inline ControllerOutputLight
fromControllerOutput(const ControllerOutput& out)
{
	ControllerOutputLight con;
	con.rollOutput = static_cast<float>(out.rollOutput);
	con.pitchOutput = static_cast<float>(out.pitchOutput);
	con.yawOutput = static_cast<float>(out.yawOutput);
	con.collectiveOutput = static_cast<float>(out.collectiveOutput);
	con.throttleOutput = static_cast<float>(out.throttleOutput);
	con.flapOutput = static_cast<float>(out.flapOutput);
	con.sequenceNr = out.sequenceNr;
	return con;
}

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, ControllerOutput& t)
{
	ar & t.rollOutput;
	ar & t.pitchOutput;
	ar & t.yawOutput;
	ar & t.collectiveOutput;
	ar & t.throttleOutput;
	ar & t.flapOutput;
	ar & t.sequenceNr;
}

template<class Archive, typename Type>
inline void
serialize(Archive& ar, ControllerOutputLight& t)
{
	ar & t.rollOutput;
	ar & t.pitchOutput;
	ar & t.yawOutput;
	ar & t.collectiveOutput;
	ar & t.throttleOutput;
	ar & t.flapOutput;
	ar & t.sequenceNr;
}

}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_CONTROLLEROUTPUT_H_ */
