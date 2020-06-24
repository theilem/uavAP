/**
 *  @file         IDCComm.cpp
 *  @author Mirco Theile
 *  @date      30 July 2017
 *  @brief      UAV Autopilot Communication Serial Comm Source File
 *
 *  Description
 */

#include <uavAP/Communication/Comm/IDCComm/IDCComm.h>
#include <uavAP/Core/DataHandling/Content.hpp>
#include <cpsCore/Utilities/IDC/IDC.h>
#include <cpsCore/Utilities/IPC/IPC.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <cpsCore/Utilities/Scheduler/IScheduler.h>

IDCComm::IDCComm() :
		senderAvailable_(true)
{
}

bool
IDCComm::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			if (!checkIsSet<IPC, IDC, DataPresentation>())
			{
				CPSLOG_ERROR << "SerialComm: missing dependency";
				return true;
			}

			auto ipc = get<IPC>();

			for (int k = static_cast<int>(Target::INVALID) + 1;
				 k < static_cast<int>(Target::COMMUNICATION); k++)
			{
				publishers_.push_back(
						ipc->publishPackets(
								"comm_to_" + EnumMap<Target>::convert(static_cast<Target>(k))));
			}

			auto idc = get<IDC>();
			sender_ = idc->createSender("ground_station");

			break;
		}
		case RunStage::NORMAL:
		{

			auto idc = get<IDC>();

			idcConnection_ = idc->subscribeOnPacket("ground_station",
													std::bind(&IDCComm::receivePacket, this, std::placeholders::_1));
			auto ipc = get<IPC>();

			IPCOptions options;
			options.multiTarget = true;
			options.retry = true;

			subscriptions_ = std::vector<Subscription>(static_cast<int>(Target::COMMUNICATION) - 1);
			for (int k = static_cast<int>(Target::INVALID) + 1;
				 k < static_cast<int>(Target::COMMUNICATION); k++)
			{
				options.retrySuccessCallback = std::bind(&IDCComm::subscribeCallback, this,
														 std::placeholders::_1, static_cast<Target>(k));

				subscriptions_[k - 1] = ipc->subscribeOnPackets(
						EnumMap<Target>::convert(static_cast<Target>(k)) + "_to_comm",
						std::bind(&IDCComm::sendPacket, this, std::placeholders::_1), options);

				if (!subscriptions_[k - 1].connected())
				{
					CPSLOG_DEBUG << EnumMap<Target>::convert(static_cast<Target>(k))
								 << " not found. Retry later.";
				}
			}
			break;
		}
		case RunStage::FINAL:
		{
			CPSLOG_DEBUG << "Run stage final";
			break;
		}
		default:
		{
			break;
		}
	}

	return false;
}

void
IDCComm::sendPacket(const Packet& packet)
{
	LockGuard lock(senderMutex_);
	sender_.sendPacket(packet);
}

void
IDCComm::receivePacket(const Packet& packet)
{
	auto dp = get<DataPresentation>();
	if (!dp)
	{
		CPSLOG_ERROR << "Data Presentation missing. Cannot handle receive.";
		return;
	}

	Packet p = packet;
	Target target = dp->extractHeader<Target>(p);

	if (target == Target::BROADCAST)
	{
		for (auto& pub : publishers_)
		{
			pub.publish(p);
		}
	}
	else if (target == Target::INVALID || target == Target::COMMUNICATION)
	{
		CPSLOG_ERROR << "Invalid Target";
	}
	else
	{
		publishers_[static_cast<int>(target) - 1].publish(p);
	}

}

void
IDCComm::subscribeCallback(const Subscription& sub, Target target)
{
	CPSLOG_DEBUG << "Subscribed to " << EnumMap<Target>::convert(target);
	subscriptions_[static_cast<int>(target) - 1] = sub;
}
