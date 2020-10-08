//
// Created by mirco on 28.08.20.
//

#ifndef UAVAP_LINEARSENSORMANAGERPARAMS_H
#define UAVAP_LINEARSENSORMANAGERPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>
#include <uavAP/FlightControl/Controller/ControlElements/Filter/LowPassFilterParams.h>

struct LinearSensorParams
{
	Parameter<unsigned> channel = {0, "channel", true};
	Parameter<FloatingType> offset = {0., "offset", true};
	Parameter<FloatingType> slope = {1., "slope", true};
	Parameter<Optional<Control::LowPassFilterParams>> filter = {{}, "filter", false};

	template <typename Config>
	inline void
	configure(Config& c)
	{
		c & channel;
		c & offset;
		c & slope;
		c & filter;
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
