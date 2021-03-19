//
// Created by mirco on 19.03.21.
//


#include <cpsCore/Utilities/TimeProvider/ITimeProvider.h>
#include "uavAP/FlightAnalysis/SignalGenerator/CustomSignalGenerator.h"


void
CustomSignalGenerator::initialize()
{
	auto tp = get<ITimeProvider>();
	if (!tp)
	{
		CPSLOG_ERROR << "Time Provider missing";
		return;
	}
	if (params.timepoints().size() != params.values().size())
	{
		CPSLOG_ERROR << "Number of timepoints doesn't match number of Values: " << params.timepoints().size() << "!="
					 << params.values().size();
	}
	init_ = tp->now();
}

FloatingType
CustomSignalGenerator::getValue()
{
	auto tp = get<ITimeProvider>();
	if (!tp)
	{
		CPSLOG_ERROR << "Time Provider missing";
		return 0;
	}
	auto deltaT = std::chrono::duration_cast<Milliseconds>(tp->now() - init_).count();
	for (std::size_t k = 1; k < params.timepoints().size(); k++)
	{
		if (params.timepoints()[k] > deltaT)
			return params.values()[k-1];
	}
	return 0.0;
}
