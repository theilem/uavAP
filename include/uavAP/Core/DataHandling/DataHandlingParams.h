/*
 * DataHandlingParams.h
 *
 *  Created on: Jul 8, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_DATAHANDLING_DATAHANDLINGPARAMS_H_
#define UAVAP_CORE_DATAHANDLING_DATAHANDLINGPARAMS_H_

#include <cpsCore/Configuration/Parameter.hpp>
#include "uavAP/Core/DataHandling/Content.hpp"



struct DataHandlingParams
{
	Parameter<int> period = {100, "period", true};
	Parameter<bool> useIPC = {true, "use_ipc", false};
	Parameter<Target> target = {Target::BROADCAST, "target", false};
	Parameter<bool> useIDC = {false, "use_idc", false};
	Parameter<std::string> idcTarget = {"autopilot", "idc_target", false};

	//Adaptive period scheduling
	Parameter<bool> useAdaptivePeriod = {false, "use_adaptive_period", false};
	Parameter<int> minPeriod = {100, "min_period", false};
	Parameter<int> maxPeriod = {1000, "max_period", false};
	Parameter<FloatingType> increment = {1.1, "increment", false};
	Parameter<FloatingType> decrement = {0.99, "decrement", false};

	template <typename Config>
	inline void
	configure(Config& c)
	{
		c & period;
		c & target;
		c & useIPC;
		c & useIDC;
		c & idcTarget;

		c & useAdaptivePeriod;
		c & minPeriod;
		c & maxPeriod;
		c & increment;
		c & decrement;
	}
};



#endif /* UAVAP_CORE_DATAHANDLING_DATAHANDLINGPARAMS_H_ */
