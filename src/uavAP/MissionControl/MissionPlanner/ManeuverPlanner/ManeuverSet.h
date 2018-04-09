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
 * Maneuver.h
 *
 *  Created on: Sep 16, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_MISSIONPLANNER_MANEUVERPLANNER_MANEUVERSET_H_
#define UAVAP_MISSIONCONTROL_MISSIONPLANNER_MANEUVERPLANNER_MANEUVERSET_H_
#include <boost/property_tree/ptree.hpp>
#include "uavAP/Core/Time.h"
#include <vector>

struct ManeuverSet
{
	std::vector<double> rollTargets;
	std::vector<double> pitchTargets;
	std::vector<double> velocityTargets;

	Duration maneuverDuration;

	bool
	configure(const boost::property_tree::ptree& config);
};



#endif /* UAVAP_MISSIONCONTROL_MISSIONPLANNER_MANEUVERPLANNER_MANEUVERSET_H_ */
