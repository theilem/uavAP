/*
 * DataHandling.cpp
 *
 *  Created on: Feb 14, 2019
 *      Author: mirco
 */
#include <uavAP/Core/DataHandling/DataHandling.h>
#include <uavAP/Core/IPC/IPC.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>
#include <uavAP/Core/DataPresentation/DataPresentation.h>

DataHandling::DataHandling()
{
}

bool
DataHandling::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!isSet<DataPresentation>())
		{
			APLOG_ERROR << "DataHandling: DataPresentation missing";
			return true;
		}

		if (!isSet<IScheduler>())
		{
			APLOG_ERROR << "DataHandling: Scheduler missing";
			return true;
		}

		if (!isSet<IPC>())
		{
			APLOG_ERROR << "DataHandling: IPC missing";
			return true;
		}

		std::string publication = EnumMap<Target>::convert(params.target()) + "_to_comm";
		APLOG_DEBUG << "Publishing to " << publication;
		auto ipc = get<IPC>();

		publisher_ = ipc->publishPackets(publication);
		break;
	}
	case RunStage::NORMAL:
	{
		auto scheduler = get<IScheduler>();
		scheduler->schedule(std::bind(&DataHandling::sendStatus, this), Milliseconds(0), Milliseconds(params.period()));

		std::string subscription = "comm_to_" + EnumMap<Target>::convert(params.target());
		auto ipc = get<IPC>();

		ipc->subscribeOnPackets(subscription,
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
	auto dp = get<DataPresentation>();
	if (!dp)
	{
		APLOG_ERROR << "DataPresentation missing";
		return;
	}
	auto p = packet;
	Content content = dp->extractHeader<Content>(p);

	auto it = subscribers_.find(content);
	if (it == subscribers_.end())
	{
		APLOG_WARN << "Packet with content " << static_cast<int>(content)
				<< " received, but no subscribers";
		return;
	}

	for (const auto& k : it->second)
	{
		k(p);
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
