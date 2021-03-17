//
// Created by mirco on 25.02.21.
//

#include <cpsCore/Framework/StaticFactory.h>
#include "uavAP/FlightAnalysis/ManeuverPlanner/Maneuver.h"
#include "uavAP/FlightAnalysis/Condition/ConditionFactory.h"
#include "uavAP/FlightAnalysis/SignalGenerator/SignalGeneratorFactory.h"
// !This has to be included such that every forward declaration in the conditions is complete for aggregation
#include "uavAP/FlightAnalysis/FlightAnalysisHelper.h"

bool
Maneuver::initialize(const Aggregator& aggregator)
{
	transition_ = ConditionFactory::create(params.transition());
	if (!transition_)
	{
		CPSLOG_ERROR << "Transition cannot be created from configuration";
		return false;
	}
	if (auto aggregatable = std::dynamic_pointer_cast<IAggregatableObject>(transition_))
		aggregatable->notifyAggregationOnUpdate(aggregator);
	transition_->initialize();

	for (const auto& p : params.waveforms())
	{
		auto signalGenerator = SignalGeneratorFactory::create(p.second);
		if (!signalGenerator)
		{
			CPSLOG_ERROR << "signalGenerator cannot be created from configuration";
			return false;
		}
		if (auto aggregatable = std::dynamic_pointer_cast<IAggregatableObject>(signalGenerator))
			aggregatable->notifyAggregationOnUpdate(aggregator);
		signalGenerator->initialize();
		waveforms_.insert(std::make_pair(p.first, signalGenerator));
	}

	return true;
}

std::map<std::string, FloatingType>
Maneuver::getOverrides() const
{
	auto overrides = params.overrides();

	for (const auto&[id,waveform] : waveforms_)
	{
		overrides.insert(std::make_pair(id, waveform->getValue()));
	}

	return overrides;
}


bool
Maneuver::inTransition()
{
	return transition_->evaluate();
}

void
Maneuver::printInfo()
{
	std::cout << "Overrides: " << std::endl;
	for (const auto& it : params.overrides())
	{
		std::cout << it.first << ": " << it.second << std::endl;
	}

	std::cout << std::endl;
	std::cout << "Transition:" << std::endl;
	transition_->printInfo();
	std::cout << std::endl;

}

bool
Maneuver::isTimeVarying() const
{
	return !waveforms_.empty();
}