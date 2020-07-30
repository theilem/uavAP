/*
 * ManeuverLocalPlannerParams.h
 *
 *  Created on: Jun 28, 2019
 *      Author: mirco
 */

#ifndef UAVAP_FLIGHTCONTROL_LOCALPLANNER_MANEUVERLOCALPLANNER_MANEUVERLOCALPLANNERPARAMS_H_
#define UAVAP_FLIGHTCONTROL_LOCALPLANNER_MANEUVERLOCALPLANNER_MANEUVERLOCALPLANNERPARAMS_H_

#include <cpsCore/Configuration/Parameter.hpp>
#include <cpsCore/Utilities/LinearAlgebra.h>

struct ManeuverLocalPlannerParams
{
	Parameter<FloatingType> kAltitude = {1.0, "k_altitude", true};
	Parameter<FloatingType> kConvergence = {1.0, "k_convergence", true};
	Parameter<FloatingType> kYawRate = {1.0, "k_yaw_rate", true};
	Parameter<FloatingType> safetyVelocity = {25.0, "safety_velocity", true};
	Parameter<FloatingType> safetyYawRate = {0.0, "safety_yaw_rate", false};
	Parameter<int> period = {0, "period", false};
	Parameter<FloatingType> overrideVelocity = {25.0, "override_velocity", false};
	Parameter<bool> doOverrideVelocity = {false, "do_override_velocity", false};

	template <class Configurator>
	inline void
	configure(Configurator& c)
	{
		c & kAltitude;
		c & kConvergence;
		c & kYawRate;
		c & safetyVelocity;
		c & safetyYawRate;
		c & period;
		c & overrideVelocity;
		c & doOverrideVelocity;
	}
};




#endif /* UAVAP_FLIGHTCONTROL_LOCALPLANNER_MANEUVERLOCALPLANNER_MANEUVERLOCALPLANNERPARAMS_H_ */
