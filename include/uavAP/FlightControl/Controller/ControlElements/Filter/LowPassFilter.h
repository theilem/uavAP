/*
 * LowPassFilter.h
 *
 *  Created on: Aug 15, 2019
 *      Author: mirco
 */

#ifndef SENSORS_FILTER_LOWPASSFILTER_H_
#define SENSORS_FILTER_LOWPASSFILTER_H_

#include <cmath>
#include <cpsCore/Configuration/ConfigurableObject.hpp>
#include "uavAP/FlightControl/Controller/ControlElements/Filter/LowPassFilterParams.h"
#include "uavAP/FlightControl/Controller/ControlElements/IEvaluableControlElement.h"

namespace Control
{

class LowPassFilter : public ConfigurableObject<LowPassFilterParams>, public IEvaluableControlElement
{
public:
	//constructors
	LowPassFilter();

	LowPassFilter(Element in, const Duration* timeDiff);

	void
	evaluate() override;

	FloatingType
	update(FloatingType input, FloatingType deltaTime);

	//get and set funtions
	FloatingType
	getValue() const override;

private:

	Element input_;

	const Duration* timeDiff_;

	FloatingType output_;
};

template <typename Type>
class LowPassFilterGeneric : public ConfigurableObject<LowPassFilterParams>
{
public:
	//constructors
	LowPassFilterGeneric() = default;

	inline Type
	update(const Type& input, Duration deltaTime)
	{

		FloatingType ePow = 1 - std::exp(-std::chrono::duration_cast<Microseconds>(deltaTime).count() / 1e6 * params.cutOffFrequency());
		output_ += (input - output_) * ePow;
		return output_;
	}

	//get and set funtions
	inline Type
	getValue() const
	{
		return output_;
	}

private:

	Type output_;
};

}

#endif /* SENSORS_FILTER_LOWPASSFILTER_H_ */
