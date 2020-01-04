/*
 * LowPassFilterAngle.h
 *
 *  Created on: Aug 15, 2019
 *      Author: mirco
 */

#ifndef SENSORS_FILTER_LOWPASSFILTERANGLE_H_
#define SENSORS_FILTER_LOWPASSFILTERANGLE_H_

#include <cpsCore/Configuration/ConfigurableObject.hpp>

#include "uavAP/FlightControl/Controller/ControlElements/Filter/LowPassFilterParams.h"
#include "uavAP/FlightControl/Controller/ControlElements/IEvaluableControlElement.h"

namespace Control
{


class LowPassFilterAngle : public ConfigurableObject<LowPassFilterParams>, public IEvaluableControlElement
{
public:
	//constructors
	LowPassFilterAngle();

	//functions

	FloatingType
	update(FloatingType input, FloatingType deltaTime);

	//get and set funtions
	FloatingType
	getValue() const override;

private:

	FloatingType output_;
};

}


#endif /* SENSORS_FILTER_LOWPASSFILTERANGLE_H_ */
