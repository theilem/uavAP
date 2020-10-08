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
	Parameter<FloatingType> cutOffFrequency = {1, "cut_off_frequency", true};
	Parameter<Optional<FloatingType>> samplingPeriod = {10, "sampling_period_ms", false};
	Parameter<Optional<FloatingType>> minValue = {std::nullopt, "min_value", false};
	Parameter<Optional<FloatingType>> maxValue = {std::nullopt, "max_value", false};

	template <typename Config>
	void
	configure(Config& c)
	{
		c & cutOffFrequency;
		c & samplingPeriod;
		c & minValue;
		c & maxValue;
	}
};

}
#endif /* SENSORS_FILTER_LOWPASSFILTERPARAMS_H_ */
