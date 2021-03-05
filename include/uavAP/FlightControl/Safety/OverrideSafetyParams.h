//
// Created by mirco on 25.02.21.
//

#ifndef UAVAP_OVERRIDESAFETYPARAMS_H
#define UAVAP_OVERRIDESAFETYPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>
#include "uavAP/Core/Rectanguloid.h"

struct OverrideSafetyParams
{
	Parameter<Rectanguloid> rectanguloid = {{}, "rectanguloid", true};
	Parameter<int> period = {10, "period", true};

	template <typename Config>
	void
	configure(Config& c)
	{
		c & rectanguloid;
		c & period;
	}
};

#endif //UAVAP_OVERRIDESAFETYPARAMS_H
