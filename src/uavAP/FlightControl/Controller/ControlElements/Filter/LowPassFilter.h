/*
 * LowPassFilter.h
 *
 *  Created on: Aug 15, 2019
 *      Author: mirco
 */

#ifndef SENSORS_FILTER_LOWPASSFILTER_H_
#define SENSORS_FILTER_LOWPASSFILTER_H_

#include <uavAP/Core/PropertyMapper/ConfigurableObject.hpp>
#include <uavAP/Core/Time.h>
#include <uavAP/FlightControl/Controller/ControlElements/Filter/LowPassFilterParams.h>
#include <uavAP/FlightControl/Controller/ControlElements/IEvaluableControlElement.h>
#include <cmath>

namespace Control
{

class LowPassFilter : public ConfigurableObject<LowPassFilterParams>, public IEvaluableControlElement
{
public:
	//constructors
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

}

#endif /* SENSORS_FILTER_LOWPASSFILTER_H_ */
