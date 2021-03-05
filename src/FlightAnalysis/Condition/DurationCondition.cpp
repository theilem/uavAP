/*
 * DurationCondition.cpp
 *
 *  Created on: Feb 26, 2021
 *      Author: Mirco Theile
 */

#include <cpsCore/Utilities/TimeProvider/ITimeProvider.h>
#include <iostream>

#include "uavAP/FlightAnalysis/Condition/DurationCondition.h"

void
DurationCondition::initialize()
{
	auto tp = get<ITimeProvider>();
	if (!tp)
	{
		CPSLOG_ERROR << "Time provider missing";
		return;
	}
	init_ = tp->now();
}

bool
DurationCondition::evaluate()
{
	auto tp = get<ITimeProvider>();
	if (!tp)
	{
		CPSLOG_ERROR << "Time provider missing";
		return true;
	}
	return std::chrono::duration_cast<Milliseconds>(tp->now() - init_).count() >= params.duration();
}

void
DurationCondition::printInfo()
{
	std::cout << "Duration Condition: " << params.duration() << " ms" << std::endl;
}
