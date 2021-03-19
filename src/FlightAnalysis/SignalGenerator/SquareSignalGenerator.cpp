//
// Created by mirco on 19.03.21.
//

#include <cpsCore/Utilities/TimeProvider/ITimeProvider.h>
#include "uavAP/FlightAnalysis/SignalGenerator/SquareSignalGenerator.h"

void
SquareSignalGenerator::initialize()
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
SquareSignalGenerator::getValue()
{
	auto tp = get<ITimeProvider>();
	if (!tp)
	{
		CPSLOG_ERROR << "Time Provider missing";
		return 0;
	}
	double t = std::chrono::duration_cast<Milliseconds>(tp->now() - init_).count() / 1000.0;
	double sineValue = std::sin(2 * M_PI * params.frequency() * t + params.phase());
	double sign = sineValue >= 0 ? 1.0 : -1.0;
	return params.amplitude() * sign + params.offset();
}
