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
 * Mission.cpp
 *
 *  Created on: Sep 18, 2017
 *      Author: mircot
 */
#include <boost/property_tree/ptree.hpp>
#include <cpsCore/Configuration/PropertyMapper.hpp>
#include "uavAP/MissionControl/MissionPlanner/Mission.h"

//bool
//Waypoint::configure(const Configuration& config)
//{
//	PropertyMapper<Configuration> pm(config);
//	pm.add<double>("n", location[1], true);
//	pm.add<double>("e", location[0], true);
//	if (!pm.add<double>("d", location[2], false))
//		pm.add<double>("u", location[2], true);
//	else
//		location[2] *= -1;
//
//	Configuration directionConfig;
//
//	if (pm.add("direction", directionConfig, false))
//	{
//		Vector3 dir;
//		PropertyMapper<Configuration> pmDir(directionConfig);
//		pmDir.add<double>("n", dir[1], true);
//		pmDir.add<double>("e", dir[0], true);
//		if (!pmDir.add<double>("d", dir[2], false))
//			pmDir.add<double>("u", dir[2], true);
//		else
//			dir[2] *= -1;
//		if (pmDir.map())
//			direction = dir;
//	}
//
//	double vel = DEFAULT_VELOCITY;
//	if (pm.add<double>("velocity", vel, false))
//		velocity = vel;
//
//	return pm.map();
//
//}
//
//bool
//Mission::configure(const Configuration& config)
//{
//	PropertyMapper<Configuration> pm(config);
//	pm.add<double>("velocity", velocity, true);
//	pm.add<bool>("infinite", infinite, true);
//
//	Waypoint offset;
//	bool hasOffset = false;
//	Configuration offsetWaypoint;
//	if (pm.add("offset", offsetWaypoint, false))
//	{
//		if (offset.configure(offsetWaypoint))
//			hasOffset = true;
//	}
//
//	Configuration waypointConfig;
//	if (pm.add("waypoints", waypointConfig, true))
//	{
//		for (auto& it : waypointConfig)
//		{
//			Waypoint wp;
//			if (wp.configure(it.second))
//			{
//				if (hasOffset)
//					wp.location = wp.location + offset.location;
//				waypoints.push_back(wp);
//			}
//		}
//	}
//
//	return pm.map();
//}
