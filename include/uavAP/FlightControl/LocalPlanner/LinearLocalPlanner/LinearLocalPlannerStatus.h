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
 * LinearLocalPlannerStatus.h
 *
 *  Created on: Aug 19, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_LOCALPLANNER_LINEARLOCALPLANNER_LINEARLOCALPLANNERSTATUS_H_
#define UAVAP_FLIGHTCONTROL_LOCALPLANNER_LINEARLOCALPLANNER_LINEARLOCALPLANNERSTATUS_H_

#include "cpsCore/Utilities/LinearAlgebra.h"
#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include <memory>
#include <cpsCore/Utilities/DataPresentation/detail/SerializeCustom.h>

struct LinearLocalPlannerStatus : public SerializeCustom
{
	FloatingType headingTarget;
	uint8_t currentPathSection;
	Vector3 velocityTarget;
	FloatingType yawRateTarget;
	bool isInApproach;
};

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, LinearLocalPlannerStatus& t)
{
	ar & t.headingTarget;
	ar & t.currentPathSection;
	ar & t.velocityTarget;
	ar & t.yawRateTarget;
	ar & t.isInApproach;
}
}

#endif /* UAVAP_FLIGHTCONTROL_LOCALPLANNER_LINEARLOCALPLANNER_LINEARLOCALPLANNERSTATUS_H_ */
