/*
 * DataHandling.cpp
 *
 *  Created on: Feb 14, 2019
 *      Author: mirco
 */
#include <uavAP/Core/DataHandling/DataHandling.h>
#include <uavAP/Core/IPC/IPC.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>

DataHandling::DataHandling() :
		target_(Target::INVALID), period_(Milliseconds(10))
{
}

std::shared_ptr<DataHandling>
DataHandling::create(const Configuration& config)
{
	auto dh = std::make_shared<DataHandling>();
	if (!dh->configure(config))
	{
		APLOG_ERROR << "DataHandling configuration failed";
	}
	return dh;
}

bool
DataHandling::configure(const Configuration& config)
{
	PropertyMapper pm(config);
	pm.add("period", period_, true);
	pm.addEnum("target", target_, true);
	return pm.map();
}

void
DataHandling::notifyAggregationOnUpdate(const Aggregator& agg)
{
	dataPresentation_.setFromAggregationIfNotSet(agg);
	scheduler_.setFromAggregationIfNotSet(agg);
	ipc_.setFromAggregationIfNotSet(agg);
}

bool
DataHandling::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!dataPresentation_.isSet())
		{
			APLOG_ERROR << "DataHandling: DataPresentation missing";
			return true;
		}

		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "DataHandling: Scheduler missing";
			return true;
		}

		if (!ipc_.isSet())
		{
			APLOG_ERROR << "DataHandling: IPC missing";
			return true;
		}

		std::string publication = EnumMap<Target>::convert(target_) + "_to_comm";
		APLOG_DEBUG << "Publishing to " << publication;
		auto ipc = ipc_.get();

		publisher_ = ipc->publishPackets(publication);
		break;
	}
	case RunStage::NORMAL:
	{
		auto scheduler = scheduler_.get();
		scheduler->schedule(std::bind(&DataHandling::sendStatus, this), Milliseconds(0), period_);

		std::string subscription = "comm_to_" + EnumMap<Target>::convert(target_);
		auto ipc = ipc_.get();

		ipc->subscribeOnPacket(subscription,
				std::bind(&DataHandling::onPacket, this, std::placeholders::_1));
		break;
	}
	default:
		break;
	}
	return false;
}

void
DataHandling::onPacket(const Packet& packet)
{
	auto dp = dataPresentation_.get();
	if (!dp)
	{
		APLOG_ERROR << "DataPresentation missing";
		return;
	}
	Content content;
	auto any = dp->deserialize(packet, content);

	auto it = subscribers_.find(content);
	if (it == subscribers_.end())
	{
		APLOG_WARN << "Packet with content " << static_cast<int>(content)
				<< " received, but no subscribers";
		return;
	}

	for (const auto& k : it->second)
	{
		k(any);
	}
}

void
DataHandling::sendStatus()
{
	for (const auto& it : statusPackaging_)
	{
		publisher_.publish(it());
	}
}
