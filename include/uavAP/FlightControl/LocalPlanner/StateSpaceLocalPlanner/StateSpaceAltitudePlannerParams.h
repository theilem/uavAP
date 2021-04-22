//
// Created by seedship on 4/19/21.
//

#ifndef UAVAP_STATESPACEALTITUDEPLANNERPARAMS_H
#define UAVAP_STATESPACEALTITUDEPLANNERPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>

struct StateSpaceAltitudePlannerParams
{
	Parameter<VectorN<7>> k = {{}, "k", true};
	Parameter<FloatingType> kConvergence = {1.0, "k_convergence", true};
	Parameter<FloatingType> kYawRate = {1.0, "k_yaw_rate", true};
	Parameter<FloatingType> yawRateDistanceThreshold = {50.0, "yaw_rate_distance_threshold", true};
	Parameter<FloatingType> safetyVelocity = {55.0, "safety_velocity", true};
	Parameter<FloatingType> safetyYawRate = {0.0, "safety_yaw_rate", false};
	Parameter<FloatingType> safetyAltitude = {2500.0, "safety_altitude", true};
	Parameter<int> period = {0, "period", false};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & k;
		c & kConvergence;
		c & kYawRate;
		c & yawRateDistanceThreshold;
		c & safetyVelocity;
		c & safetyYawRate;
		c & safetyAltitude;
		c & period;
	}
};


#endif //UAVAP_STATESPACEPLANNERALTITUDEPARAMS_H
