//
// Created by mirco on 04.01.20.
//

#ifndef UAVAP_SENSORDATACONDITIONPARAMS_H
#define UAVAP_SENSORDATACONDITIONPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>

struct SensorDataConditionParams
{

	Parameter<bool> filterData = {false, "filter_data", false};
	Parameter<bool> useTolerance = {false, "use_tolerance", false};
	Parameter<SensorEnum> sensor = {SensorEnum::INVALID, "sensor", true};
	Parameter<Relational> relational = {Relational::INVALID, "relational", true};
	Parameter<double> tolerance = {0.0, "tolerance", false};
	Parameter<double> threshold = {0.0, "threshold", true};


	template <typename Config>
	void
	configure(Config& c)
	{
		c & filterData;
		c & useTolerance;
		c & sensor;
		c & relational;
		c & tolerance;
		c & threshold;
	}
};


#endif //UAVAP_SENSORDATACONDITIONPARAMS_H
