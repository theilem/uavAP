//
// Created by mirco on 19.03.21.
//

#ifndef UAVAP_PERIODICSIGNALPARAMS_H
#define UAVAP_PERIODICSIGNALPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>

struct PeriodicSignalParams
{
	Parameter<FloatingType> amplitude = {1.0, "amplitude", true};
	Parameter<FloatingType> frequency = {1.0, "frequency", true};
	Parameter<Angle<FloatingType>> phase = {{}, "phase", false};
	Parameter<FloatingType> offset = {0.0, "offset", false};

	template<typename Config>
	void
	configure(Config& c)
	{
		c & amplitude;
		c & frequency;
		c & phase;
		c & offset;
	}
};

#endif //UAVAP_PERIODICSIGNALPARAMS_H
