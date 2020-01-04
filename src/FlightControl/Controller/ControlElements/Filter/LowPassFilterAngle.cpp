/*
 * LowPassFilterAngle.cpp
 *
 *  Created on: Aug 15, 2019
 *      Author: mirco
 */

#include "uavAP/FlightControl/Controller/ControlElements/Filter/LowPassFilterAngle.h"

namespace Control
{
LowPassFilterAngle::LowPassFilterAngle() :
		output_(0), ePow_(0)
{
}

FloatingType
LowPassFilterAngle::update(FloatingType input)
{
	FloatingType cosa = cos(input) + ePow_ * (cos(input) - cos(output_));
	FloatingType sina = sin(input) + ePow_ * (sin(input) - sin(output_));

	output_ = atan2(sina, cosa);
	return output_;
}

FloatingType
LowPassFilterAngle::update(FloatingType input, FloatingType deltaTimeSec)
{
	FloatingType ePow = 1 - exp(-deltaTimeSec * params.cutOffFrequency());

	FloatingType cosa = cos(input) + ePow * (cos(input) - cos(output_));
	FloatingType sina = sin(input) + ePow * (sin(input) - sin(output_));

	output_ = atan2(sina, cosa);
	return output_;
}

FloatingType
LowPassFilterAngle::getValue() const
{
	return output_;
}

void
LowPassFilterAngle::init()
{
	ePow_ = 1 - exp(-static_cast<FloatingType>(params.deltaTime()) / 1000.0 * params.cutOffFrequency());
}
}
