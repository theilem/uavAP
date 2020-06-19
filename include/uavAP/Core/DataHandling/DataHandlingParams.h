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

	template <typename Config>
	inline void
	configure(Config& c)
	{
		c & period;
		c & target;
		c & useIPC;
		c & useIDC;
		c & idcTarget;
	}
};



#endif /* UAVAP_CORE_DATAHANDLING_DATAHANDLINGPARAMS_H_ */
