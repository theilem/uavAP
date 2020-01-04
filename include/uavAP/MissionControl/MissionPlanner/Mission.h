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
#include <cpsCore/Utilities/LinearAlgebra.h>
#include <cpsCore/Utilities/Optional.hpp>
#include <cpsCore/Configuration/Configuration.hpp>


#define DEFAULT_VELOCITY 15.0

struct Waypoint
{
	Vector3 location;
	Optional<Vector3> direction;
	Optional<FloatingType> velocity; //Desired velocity upto this waypoint

	Waypoint() :
			location(0, 0, 0), direction(), velocity()
	{
	}

	Waypoint(const Vector3& loc) :
			location(loc), direction(), velocity()
	{
	}

	Waypoint(const Vector3& loc, const Vector3& dir) :
			location(loc), direction(dir), velocity()
	{
	}

	Waypoint(const Vector3& loc, FloatingType vel) :
			location(loc), direction(), velocity(vel)
	{
	}

	Waypoint(const Vector3& loc, const Vector3& dir, FloatingType vel) :
			location(loc), direction(dir), velocity(vel)
	{
	}

	bool
	configure(const Configuration& config);

};

struct Mission
{
	std::vector<Waypoint> waypoints;
	Optional<Waypoint> initialPosition;
	bool infinite;
	FloatingType velocity;

	Mission() :
			infinite(false), velocity(DEFAULT_VELOCITY)
	{
	}

	bool
	configure(const Configuration& config);
};

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, Waypoint& t)
{
	ar & t.location;
	ar & t.velocity;
	ar & t.direction;
}

template<class Archive, typename Type>
inline void
serialize(Archive& ar, Mission& t)
{
	ar & t.waypoints;
	ar & t.initialPosition;
	ar & t.velocity;
	ar & t.infinite;
}
}

#endif /* UAVAP_MISSIONCONTROL_MISSION_H_ */
