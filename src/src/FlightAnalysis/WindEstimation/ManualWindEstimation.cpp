/*
 * ManualWindEstimation.cpp
 *
 *  Created on: Feb 8, 2019
 *      Author: mirco
 */
#include <uavAP/FlightAnalysis/WindEstimation/ManualWindEstimation.h>


bool
ManualWindEstimation::configure(const Configuration& config)
{
	return true;
}

std::shared_ptr<ManualWindEstimation>
ManualWindEstimation::create(const Configuration& config)
{
	auto mwe = std::make_shared<ManualWindEstimation>();
	if (!mwe->configure(config))
		APLOG_ERROR << "ManualWindEstimation failed to configure";

	return mwe;
}

void
ManualWindEstimation::notifyAggregationOnUpdate(const Aggregator& agg)
{
	ipc_.setFromAggregationIfNotSet(agg);
}

bool
ManualWindEstimation::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!ipc_.isSet())
		{
			APLOG_ERROR << "ManualWindEstimation ipc missing.";
			return true;
		}
		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = ipc_.get();
		ipc->subscribeOnPacket("")
		break;
	}
	case RunStage::FINAL:
	{
		break;
	}
	default:
		break;
	}
	return false;
}

WindInfo
ManualWindEstimation::getWindInfo() const
{
}

void
ManualWindEstimation::onPacket(const Packet& packet)
{
}
