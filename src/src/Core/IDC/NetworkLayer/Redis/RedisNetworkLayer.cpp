/*
 * RedisNetworkLayer.cpp
 *
 *  Created on: Jan 25, 2019
 *      Author: mirco
 */

#include <uavAP/Core/IDC/NetworkLayer/Redis/RedisNetworkLayer.h>
#include <uavAP/Core/IDC/NetworkLayer/Redis/RedisPublisher.h>
#include <uavAP/Core/IDC/NetworkLayer/Redis/RedisSubscriber.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>
#include <uavAP/Core/Scheduler/IScheduler.h>


std::shared_ptr<RedisNetworkLayer>
RedisNetworkLayer::create(const boost::property_tree::ptree& config)
{
	auto rnl = std::make_shared<RedisNetworkLayer>();
	rnl->configure(config);
	return rnl;
}

bool
RedisNetworkLayer::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);

	boost::property_tree::ptree sub;
	pm.add("sub", sub, false);

	for (const auto& it : sub)
	{
		RedisChannelParams params;
		if (!params.configure(it.second))
			return false;

		subscribers_.emplace(it.first, std::make_shared<RedisSubscriber>(params));
	}

	boost::property_tree::ptree pub;
	pm.add("pub", pub, false);

	for (const auto& it : pub)
	{
		RedisChannelParams params;
		if (!params.configure(it.second))
			return false;

		publishers_.emplace(it.first, std::make_shared<RedisPublisher>(params));
	}

	return pm.map();

}

bool
RedisNetworkLayer::sendPacket(const std::string& id, const Packet& packet)
{
	auto it = publishers_.find(id);
	if (it == publishers_.end())
	{
		APLOG_ERROR << "Connection id not specified: " << id;
		return false;
	}

	return it->second->sendPacket(packet);
}

boost::signals2::connection
RedisNetworkLayer::subscribeOnPacket(const std::string& id, const OnPacket::slot_type& handle)
{
	auto it = subscribers_.find(id);
	if (it == subscribers_.end())
	{
		APLOG_ERROR << "Connection id not specified: " << id;
		return boost::signals2::connection();
	}

	return it->second->subscribeOnPackets(handle);
}

void
RedisNetworkLayer::notifyAggregationOnUpdate(const Aggregator& agg)
{
	scheduler_.setFromAggregationIfNotSet(agg);
}

bool
RedisNetworkLayer::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "RedisNetworkLayer scheduler missing";
			return true;
		}
		break;
	}
	case RunStage::NORMAL:
	{
		auto sched = scheduler_.get();
		for (const auto& it : subscribers_)
		{
			sched->schedule(std::bind(&RedisSubscriber::startHandler, it.second), Milliseconds(0));
		}

		for (const auto& it : publishers_)
		{
			sched->schedule(std::bind(&RedisPublisher::startHandler, it.second), Milliseconds(0));
		}
		break;
	}
	default:
		break;
	}
	return false;
}
