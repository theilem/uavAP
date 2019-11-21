//
// Created by mirco on 17.11.19.
//

#ifndef UAVAP_MICROSIMULATORPARAMS_H
#define UAVAP_MICROSIMULATORPARAMS_H

#include "uavAP/Core/PropertyMapper/Parameter.h"

struct MicroSimulatorParams
{
	Parameter<float> realTimeFactor = {0, "real_time_factor", false};

	template<typename Config>
	void
	configure(Config& c)
	{
		c & realTimeFactor;
	}
};


#endif //UAVAP_MICROSIMULATORPARAMS_H
