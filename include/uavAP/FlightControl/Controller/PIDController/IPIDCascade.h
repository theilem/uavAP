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
 * IPIDCascade.h
 *
 *  Created on: Aug 13, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_SIMPLEPIDCONTROLLER_IPIDCASCADE_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_SIMPLEPIDCONTROLLER_IPIDCASCADE_H_
#include "uavAP/FlightControl/Controller/ControlElements/EvaluableControlElements.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDMapping.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDHandling.h"
#include <map>


class IPIDCascade
{
public:
	virtual
	~IPIDCascade() = default;

	virtual bool
	tunePID(PIDs pid, const Control::PIDParameters& params) = 0;

	virtual bool
	tuneRollBounds(FloatingType min, FloatingType max) = 0;

	virtual bool
	tunePitchBounds(FloatingType min, FloatingType max) = 0;

	virtual PIDStati
	getPIDStatus() const = 0;

	virtual void
	evaluate() = 0;

};

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_SIMPLEPIDCONTROLLER_IPIDCASCADE_H_ */
