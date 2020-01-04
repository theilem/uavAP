/*
 * LowPassFilter.cpp
 *
 *  Created on: Aug 15, 2019
 *      Author: mirco
 */
#include <uavAP/FlightControl/Controller/ControlElements/Filter/LowPassFilter.h>
#include <cmath>

namespace Control
{
LowPassFilter::LowPassFilter(Element in, const Duration* timeDiff) :
		input_(in), timeDiff_(timeDiff), output_(0)
{
}

FloatingType
LowPassFilter::update(FloatingType input, FloatingType deltaTimeSec)
{
	FloatingType ePow = 1 - std::exp(-deltaTimeSec * params.cutOffFrequency());
	output_ += (input - output_) * ePow;
	return output_;
}

void
LowPassFilter::evaluate()
{
	if (!timeDiff_)
		return;

	update(input_->getValue(),
			std::chrono::duration_cast<Microseconds>(*timeDiff_).count() / MUSEC_TO_SEC);
}

FloatingType
LowPassFilter::getValue() const
{
	return output_;
}


LowPassFilter::LowPassFilter():
		input_(nullptr), timeDiff_(nullptr), output_(0)
{
}
}
