//
// Created by mirco on 09.03.21.
//

#include "uavAP/FlightAnalysis/SignalGenerator/SineSignalGenerator.h"
#include <cpsCore/Utilities/TimeProvider/ITimeProvider.h>

void
SineSignalGenerator::initialize()
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
SineSignalGenerator::getValue()
{
	auto tp = get<ITimeProvider>();
	if (!tp)
	{
		CPSLOG_ERROR << "Time Provider missing";
		return 0;
	}
	double t = std::chrono::duration_cast<Milliseconds>(tp->now() - init_).count() / 1000.0;
	return params.amplitude() * std::sin(2 * M_PI * params.frequency() * t + params.phase()) + params.offset();
}
