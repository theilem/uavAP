/*
 * DataHandling.cpp
 *
 *  Created on: Feb 14, 2019
 *      Author: mirco
 */
#include <cpsCore/Utilities/Scheduler/IScheduler.h>
#include <cpsCore/Utilities/IDC/IDC.h>

#include <uavAP/Core/DataHandling/DataHandling.h>

bool
DataHandling::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			if (!checkIsSet<DataPresentation, IScheduler>())
			{
				CPSLOG_ERROR << "DataHandling: missing deps";
				return true;
			}

			if (params.useIDC())
			{
				if (!checkIsSet<IDC>())
				{
					CPSLOG_ERROR << "DataHandling: missing deps";
					return true;
				}

				auto idc = get<IDC>();
				sender_ = idc->createSender(params.idcTarget());
			}

			if (params.useIPC())
			{
				if (!checkIsSet<IPC>())
				{
					CPSLOG_ERROR << "DataHandling: missing deps";
					return true;
				}

				std::string publication = EnumMap<Target>::convert(params.target()) + "_to_comm";
				CPSLOG_DEBUG << "Publishing to " << publication;
				auto ipc = get<IPC>();
				IPCOptions options;
				options.multiTarget = false;
				publisher_ = ipc->publishPackets(publication, options);
			}


			break;
		}
		case RunStage::NORMAL:
		{
			auto scheduler = get<IScheduler>();
			scheduler->schedule([this]
								{ sendStatus(); }, Milliseconds(0), Milliseconds(params.period()));

			if (params.useIDC())
			{
				auto idc = get<IDC>();

				idc->subscribeOnPacket(params.idcTarget(),
									   std::bind(&DataHandling::onPacket, this, std::placeholders::_1));
			}

			if (params.useIPC())
			{
				std::string subscription = "comm_to_" + EnumMap<Target>::convert(params.target());
				auto ipc = get<IPC>();

				IPCOptions options;
				options.multiTarget = false;
				ipc->subscribeOnPackets(subscription,
										std::bind(&DataHandling::onPacket, this, std::placeholders::_1), options);
			}


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
	for (const auto& packetSub: packetSubscriptions_)
	{
		packetSub(packet);
	}
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

	for (const auto& k: it->second)
	{
		k(p);
	}
}

void
DataHandling::sendStatus()
{
	for (const auto& it: statusPackaging_)
	{
		publish(it());
	}
}

void
DataHandling::publish(const Packet& packet)
{
	if (params.useIDC())
	{
		sender_.sendPacket(packet);
	}
	if (params.useIPC())
	{
		publisher_.publish(packet);
	}
}

void
DataHandling::subscribeOnPackets(const std::function<void(const Packet&)>& packetSub)
{
	packetSubscriptions_.push_back(packetSub);
}
