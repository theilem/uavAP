/*
 * LowPassFilterParams.h
 *
 *  Created on: Aug 15, 2019
 *      Author: mirco
 */

#ifndef SENSORS_FILTER_LOWPASSFILTERPARAMS_H_
#define SENSORS_FILTER_LOWPASSFILTERPARAMS_H_

#include <cpsCore/Configuration/Parameter.hpp>

namespace Control
{

struct LowPassFilterParams
{
	Parameter<float> cutOffFrequency = {1, "cut_off_frequency", true};

	template <typename Config>
	void
	configure(Config& c)
	{
		c & cutOffFrequency;
	}
};

}
#endif /* SENSORS_FILTER_LOWPASSFILTERPARAMS_H_ */
