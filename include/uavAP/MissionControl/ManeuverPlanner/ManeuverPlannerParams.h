/*
 * ManeuverPlannerParams.h
 *
 *  Created on: Jun 28, 2019
 *      Author: mirco
 */

#ifndef UAVAP_MISSIONCONTROL_MANEUVERPLANNER_MANEUVERPLANNERPARAMS_H_
#define UAVAP_MISSIONCONTROL_MANEUVERPLANNER_MANEUVERPLANNERPARAMS_H_
#include <cpsCore/Utilities/LinearAlgebra.h>
#include <cpsCore/Configuration/Parameter.hpp>
#include <cpsCore/Configuration/ParameterRef.hpp>
#include <uavAP/MissionControl/Geofencing/Rectanguloid.h>

template <typename Config>
inline void
configure(Config& c, Rectanguloid& r)
{
	ParameterRef<Vector3> center(r.center, Vector3(0,0,0), "center", true);
	ParameterRef<FloatingType> majorSideLength(r.majorSideLength, 0, "major_side_length", true);
	ParameterRef<FloatingType> minorSideLength(r.minorSideLength, 0, "minor_side_length", true);
	ParameterRef<FloatingType> majorSideOrientation(r.majorSideOrientation, 0, "major_side_orientation", true);
	ParameterRef<FloatingType> height(r.height, 0, "height", true);

	c & center;
	c & majorSideLength;
	c & minorSideLength;
	c & majorSideOrientation;
	c & height;
}


struct ManeuverPlannerParams
{
	Parameter<Rectanguloid> safetyBounds = {{}, "safety_bounds", true};
	Parameter<FloatingType> returnVelocity = {25.0, "return_velocity", true};
	Parameter<bool> manualRestart = {false, "manual_restart", false};
	Parameter<bool> maneuverRestart = {false, "maneuver_restart", false};
	Parameter<bool> useSafetyBounds = {true, "use_safety_bounds", false};
	Parameter<bool> performInSafetyBounds = {true, "perform_in_safety_bounds", false};

	template <typename Config>
	void
	configure(Config& c)
	{
		c & safetyBounds;
		c & returnVelocity;
		c & manualRestart;
		c & maneuverRestart;
		c & useSafetyBounds;
		c & performInSafetyBounds;
	}
};



#endif /* UAVAP_MISSIONCONTROL_MANEUVERPLANNER_MANEUVERPLANNERPARAMS_H_ */
