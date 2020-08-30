//
// Created by mirco on 28.08.20.
//

#ifndef UAVAP_LINEARSENSORMANAGERPARAMS_H
#define UAVAP_LINEARSENSORMANAGERPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>

struct LinearSensorParams
{
	Parameter<unsigned> channel = {0, "channel", true};
	Parameter<FloatingType> offset = {0., "offset", true};
	Parameter<FloatingType> slope = {1., "slope", true};

	template <typename Config>
	inline void
	configure(Config& c)
	{
		c & channel;
		c & offset;
		c & slope;
	}
};

struct LinearSensorManagerParams
{

	Parameter<std::map<std::string, LinearSensorParams>> sensors = {{}, "sensors", true};

	template <typename Config>
	inline void
	configure(Config& c)
	{
		c & sensors;
	}
};


#endif //UAVAP_LINEARSENSORMANAGERPARAMS_H
