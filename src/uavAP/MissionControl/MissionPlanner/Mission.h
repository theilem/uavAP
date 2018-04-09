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
 * Mission.h
 *
 *  Created on: Jun 23, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_MISSION_H_
#define UAVAP_MISSIONCONTROL_MISSION_H_

#include <vector>

#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>

#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/Core/protobuf/messages/Positions.pb.h"

#define DEFAULT_VELOCITY 15.0

struct Waypoint
{
    Vector3 location;
    boost::optional<Vector3> direction;
    boost::optional<double> velocity; //Desired velocity upto this waypoint

    Waypoint() :
        location(0, 0, 0),
        direction(boost::none),
        velocity(boost::none)
    {
    }

    Waypoint(const Vector3& loc) :
        location(loc),
        direction(boost::none),
        velocity(boost::none)
    {
    }

    Waypoint(const Vector3& loc, const Vector3& dir) :
        location(loc),
        direction(dir),
        velocity(boost::none)
    {
    }

    Waypoint(const Vector3& loc, double vel) :
        location(loc),
        direction(boost::none),
        velocity(vel)
    {
    }

    Waypoint(const Vector3& loc, const Vector3& dir, double vel) :
        location(loc),
        direction(dir),
        velocity(vel)
    {
    }

    bool
    configure(const boost::property_tree::ptree& config);

};

struct Mission
{
    std::vector<Waypoint> waypoints;
    bool infinite;
    double velocity;

    Mission():
        infinite(false),
        velocity(DEFAULT_VELOCITY)
    {
    }

    bool
    configure(const boost::property_tree::ptree& config);
};

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, Waypoint& t)
{
    ar& t.location;
    ar& t.velocity;
    ar& t.direction;
}
}

#endif /* UAVAP_MISSIONCONTROL_MISSION_H_ */
