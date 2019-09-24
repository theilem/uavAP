////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/**
 *  @file         SerialComm.cpp
 *  @author Mirco Theile
 *  @date      30 July 2017
 *  @brief      UAV Autopilot Communication Serial Comm Source File
 *
 *  Description
 */

#include <uavAP/Communication/Comm/IDCComm/IDCComm.h>
#include <uavAP/Core/DataPresentation/Content.h>
#include <uavAP/Core/IDC/IDC.h>
#include <uavAP/Core/IPC/IPC.h>
#include <uavAP/Core/Object/AggregatableObjectImpl.hpp>
#include "uavAP/Core/DataPresentation/DataPresentation.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/FlightControl/Controller/IController.h"
#include "uavAP/Core/Scheduler/IScheduler.h"
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/LockTypes.h"

bool
IDCComm::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!checkIsSet<IPC, IDC, DataPresentation>())
		{
			APLOG_ERROR << "SerialComm: missing dependency";
			return true;
		}

		auto ipc = get<IPC>();

		for (int k = static_cast<int>(Target::INVALID) + 1; k < static_cast<int>(Target::COMMUNICATION);
				k++)
		{
			publishers_.push_back(
					ipc->publishPackets(
							"comm_to_" + EnumMap<Target>::convert(static_cast<Target>(k))));
		}

		auto idc = get<IDC>();
		sender_ = idc->createSender("ground_station");

		idcConnection_ = idc->subscribeOnPacket("ground_station",
				std::bind(&IDCComm::receivePacket, this, std::placeholders::_1));

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = get<IPC>();

		IPCOptions options;
		options.multiTarget = true;
		options.retry = true;

		subscriptions_ = std::vector<Subscription>(static_cast<int>(Target::COMMUNICATION) - 1);
		for (int k = static_cast<int>(Target::INVALID) + 1; k < static_cast<int>(Target::COMMUNICATION);
				k++)
		{
			options.retrySuccessCallback = std::bind(&IDCComm::subscribeCallback, this, std::placeholders::_1, static_cast<Target>(k));

			subscriptions_[k-1] = ipc->subscribeOnPackets(EnumMap<Target>::convert(static_cast<Target>(k)) + "_to_comm",
					std::bind(&IDCComm::sendPacket, this, std::placeholders::_1), options);

			if (!subscriptions_[k-1].connected())
			{
				APLOG_DEBUG << EnumMap<Target>::convert(static_cast<Target>(k)) << " not found. Retry later.";
			}
		}
		break;
	}
	case RunStage::FINAL:
	{
		APLOG_DEBUG << "Run stage final";
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
		APLOG_ERROR << "Data Presentation missing. Cannot handle receive.";
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
		APLOG_ERROR << "Invalid Target";
	}
	else
	{
		publishers_[static_cast<int>(target) - 1].publish(p);
	}

}

void
IDCComm::subscribeCallback(const Subscription& sub, Target target)
{
	APLOG_DEBUG << "Subscribed to " << EnumMap<Target>::convert(target);
	subscriptions_[static_cast<int>(target) - 1] = sub;
}
