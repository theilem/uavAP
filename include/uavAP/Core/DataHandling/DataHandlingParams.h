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
	Parameter<Target> target = {Target::INVALID, "target", true};

	template <typename Config>
	inline void
	configure(Config& c)
	{
		c & period;
		c & target;
	}
};



#endif /* UAVAP_CORE_DATAHANDLING_DATAHANDLINGPARAMS_H_ */
