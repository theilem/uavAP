/*
 * DataHandling.cpp
 *
 *  Created on: Feb 14, 2019
 *      Author: mirco
 */
#include <cpsCore/Utilities/Scheduler/IScheduler.h>

#include <uavAP/Core/DataHandling/DataHandling.h>

bool
DataHandling::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!checkIsSet<DataPresentation, IScheduler, IPC>())
		{
			CPSLOG_ERROR << "DataHandling: missing deps";
			return true;
		}

		std::string publication = EnumMap<Target>::convert(params.target()) + "_to_comm";
		CPSLOG_DEBUG << "Publishing to " << publication;
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
		CPSLOG_ERROR << "DataPresentation missing";
		return;
	}
	auto p = packet;
	Content content = dp->extractHeader<Content>(p);

	auto it = subscribers_.find(content);
	if (it == subscribers_.end())
	{
		CPSLOG_WARN << "Packet with content " << static_cast<int>(content)
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
