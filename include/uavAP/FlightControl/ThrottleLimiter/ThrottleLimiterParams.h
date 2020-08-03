//
// Created by mirco on 03.08.20.
//

#ifndef UAVAP_THROTTLELIMITERPARAMS_H
#define UAVAP_THROTTLELIMITERPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>

struct ThrottleLimiterParams
{

	Parameter<float> throttleLimit = {1.0, "throttle_limit", false};
	Parameter<bool> applyThrottleLimit = {false, "apply_throttle_limit", false};


	template <typename Config>
	void
	configure(Config& c)
	{
		c & throttleLimit;
		c & applyThrottleLimit;
	}
};


#endif //UAVAP_THROTTLELIMITERPARAMS_H
