//
// Created by mirco on 19.03.21.
//


#include <cpsCore/Utilities/TimeProvider/ITimeProvider.h>
#include "uavAP/FlightAnalysis/SignalGenerator/MultiSineSignalGenerator.h"

void
MultiSineSignalGenerator::initialize()
{
	auto tp = get<ITimeProvider>();
	if (!tp)
	{
		CPSLOG_ERROR << "Time Provider missing";
		return;
	}
	init_ = tp->now();
}

FloatingType
MultiSineSignalGenerator::getValue()
{
	auto tp = get<ITimeProvider>();
	if (!tp)
	{
		CPSLOG_ERROR << "Time Provider missing";
		return 0;
	}
	double t = std::chrono::duration_cast<Milliseconds>(tp->now() - init_).count() / 1000.0;
	double value = 0.0;

	for (const auto& sine: params.sines())
	{
		value +=  sine.amplitude() * std::sin(2 * M_PI * sine.frequency() * t + sine.phase()) + sine.offset();
	}

	return value;
}
