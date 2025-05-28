/*
 * Mission.h
 *
 *  Created on: Jun 23, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_MISSION_H_
#define UAVAP_MISSIONCONTROL_MISSION_H_

#include <vector>
#include <optional>
#include <cpsCore/Utilities/LinearAlgebra.h>
#include <cpsCore/Configuration/Parameter.hpp>


#define DEFAULT_VELOCITY 15.0

struct Waypoint
{
	Parameter<Vector3> location = {{}, "location", true};
	Parameter<std::optional<Vector3>> direction = {std::nullopt, "direction", false};
	Parameter<std::optional<FloatingType>> velocity = {std::nullopt, "velocity", false};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & location;
		c & direction;
		c & velocity;
	}

};

struct Mission
{
	Parameter<std::vector<Waypoint>> waypoints = {{}, "waypoints", true};
	Parameter<std::optional<Vector3>> offset = {std::nullopt, "offset", false};
	Parameter<bool> infinite = {true, "infinite", true};
	Parameter<FloatingType> velocity = {DEFAULT_VELOCITY, "velocity", true};


	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & waypoints;
		c & offset;
		c & infinite;
		c & velocity;
	}
};

//namespace dp
//{
//template<class Archive, typename Type>
//inline void
//serialize(Archive& ar, Waypoint& t)
//{
//	ar & t.location;
//	ar & t.velocity;
//	ar & t.direction;
//}
//
//template<class Archive, typename Type>
//inline void
//serialize(Archive& ar, Mission& t)
//{
//	ar & t.waypoints;
//	ar & t.initialPosition;
//	ar & t.velocity;
//	ar & t.infinite;
//}
//}

#endif /* UAVAP_MISSIONCONTROL_MISSION_H_ */
