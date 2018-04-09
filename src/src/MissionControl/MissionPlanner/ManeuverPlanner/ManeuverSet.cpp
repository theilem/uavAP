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
 * Maneuver.cpp
 *
 *  Created on: Sep 16, 2017
 *      Author: mircot
 */
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/MissionControl/MissionPlanner/ManeuverPlanner/ManeuverSet.h"

void
degreeToRad(std::vector<double>& vec)
{
	for (auto& it : vec)
	{
		it = it * M_PI / 180.0;
	}
}

bool
ManeuverSet::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	pm.addVector("pitch", pitchTargets, false);
	pm.addVector("roll", rollTargets, false);
	pm.addVector("velocity", velocityTargets, false);
	pm.add("duration", maneuverDuration, true);

	degreeToRad(rollTargets);
	degreeToRad(pitchTargets);
	return pm.map();
}
