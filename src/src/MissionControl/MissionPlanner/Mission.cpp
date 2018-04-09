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
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/MissionControl/MissionPlanner/Mission.h"

bool
Waypoint::configure(const boost::property_tree::ptree& config)
{
    PropertyMapper pm(config);
    pm.add("n", location[1], true);
    pm.add("e", location[0], true);
    if (!pm.add("d", location[2], false))
        pm.add("u", location[2], true);
    else
        location[2] *= -1;

    boost::property_tree::ptree directionConfig;

    if (pm.add("direction", directionConfig, false))
    {
        Vector3 dir;
        PropertyMapper pmDir(directionConfig);
        pmDir.add("n", dir[1], true);
        pmDir.add("e", dir[0], true);
        if (!pmDir.add("d", dir[2], false))
            pmDir.add("u", dir[2], true);
        else
            dir[2] *= -1;
        if (pmDir.map())
            direction = dir;
    }

    double vel = DEFAULT_VELOCITY;
    if (pm.add("velocity", vel, false))
        velocity = vel;

    return pm.map();

}

bool
Mission::configure(const boost::property_tree::ptree& config)
{
    PropertyMapper pm(config);
    pm.add("velocity", velocity, true);
    pm.add("infinite", infinite, true);

    Waypoint offset;
    bool hasOffset = false;
    boost::property_tree::ptree offsetWaypoint;
    if (pm.add("offset", offsetWaypoint, false))
    {
        if (offset.configure(offsetWaypoint))
            hasOffset = true;
    }

    boost::property_tree::ptree waypointConfig;
    if (pm.add("waypoints", waypointConfig, true))
    {
        for (auto& it : waypointConfig)
        {
            Waypoint wp;
            if (wp.configure(it.second))
            {
                if (hasOffset)
                    wp.location = wp.location + offset.location;
                waypoints.push_back(wp);
            }
        }
    }

    return pm.map();
}














