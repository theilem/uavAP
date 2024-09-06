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
 * PIDHanding.h
 *
 *  Created on: Aug 13, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_PIDHANDLING_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_PIDHANDLING_H_

#include <map>
#include <uavAP/FlightControl/Controller/PIDController/PIDMapping.h>
#include "uavAP/FlightControl/Controller/ControlElements/EvaluableControlElements.h"

struct PIDTuning
{
	PIDs pid;
	Control::PIDParameters params;
};

struct ConstraintParams
{
	FloatingType min;
	FloatingType max;
};

struct PIDStatus
{
	FloatingType target;
	FloatingType value;
	FloatingType integrator;
};

using PIDStati = std::map<PIDs, PIDStatus>;
using TimedPIDStati = std::pair<TimePoint, PIDStati>;

using PIDParams = std::map<PIDs, Control::PIDParameters>;

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, PIDTuning& t)
{
	ar & t.pid;
	ar & t.params;
}
}
#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_PIDHANDLING_H_ */
