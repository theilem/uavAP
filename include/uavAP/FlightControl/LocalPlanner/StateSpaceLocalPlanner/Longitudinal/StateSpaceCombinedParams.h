//
// Created by seedship on 5/11/21.
//

#ifndef UAVAP_STATESPACECOMBINEDPARAMS_H
#define UAVAP_STATESPACECOMBINEDPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>
#include "StateSpaceAltitudePlannerParams.h"

struct StateSpaceCombinedParams
{
	Parameter<StateSpaceAltitudePlannerParams> stateSpaceAltitudeParams = {{}, "state_space_altitude_params", true};
	Parameter<FloatingType> kConvergence = {1.0, "k_convergence", true};
	Parameter<FloatingType> kYawRate = {1.0, "k_yaw_rate", true};
	Parameter<FloatingType> yawRateDistanceThreshold = {50.0, "yaw_rate_distance_threshold", true};
	Parameter<int> period = {0, "period", false};


	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & stateSpaceAltitudeParams;
		c & kConvergence;
		c & kYawRate;
		c & yawRateDistanceThreshold;
		c & period;
	}
};

#endif //UAVAP_STATESPACECOMBINEDPARAMS_H
