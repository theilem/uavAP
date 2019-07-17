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
 *  @file         MessageQueueComm.cpp
 *  @author Mirco Theile
 *  @date      30 July 2017
 *  @brief      UAV Autopilot Communication Serial Comm Source File
 *
 *  Description
 */

#include <uavAP/Core/DataPresentation/Content.h>
#include "uavAP/Communication/Comm/MessageQueueComm/MessageQueueComm.h"
#include "uavAP/Core/DataPresentation/DataPresentation.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/FlightControl/Controller/IController.h"
#include "uavAP/Core/Scheduler/IScheduler.h"
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/LockTypes.h"

MessageQueueComm::MessageQueueComm() :
		groundStationConnected_(false)
{
}

std::shared_ptr<MessageQueueComm>
MessageQueueComm::create(const Configuration& configuration)
{
	auto comm = std::make_shared<MessageQueueComm>();

	if (!comm->configure(configuration))
	{
		APLOG_ERROR << "MessageQueueComm: Failed to Load Global Configurations";
	}

	return comm;
}

bool
MessageQueueComm::configure(const Configuration& configuration)
{
	PropertyMapper pm(configuration);

	return pm.map();
}

bool
MessageQueueComm::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "MessageQueueComm: Scheduler missing.";
			return true;
		}
		if (!ipc_.isSet())
		{
			APLOG_ERROR << "MessageQueueComm: IPC missing";
			return true;
		}

		auto ipc = ipc_.get();
		flightAnalysisPublisher_ = ipc->publishPackets("data_com_fa");
		flightControlPublisher_ = ipc->publishPackets("data_com_fc");
		missionControlPublisher_ = ipc->publishPackets("data_com_mc");
		apiPublisher_ = ipc->publishPackets("data_com_api");

		groundStationPublisher_ = ipc->publishPackets("autopilot_groundstation");

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = ipc_.get();

		flightAnalysisSubscription_ = ipc->subscribeOnPackets("data_fa_com",
				std::bind(&MessageQueueComm::sendPacket, this, std::placeholders::_1));

		if (!flightAnalysisSubscription_.connected())
		{
			APLOG_WARN << "Cannot connect to data_fa_com. Ignoring.";
		}

		flightControlSubscription_ = ipc->subscribeOnPackets("data_fc_com",
				std::bind(&MessageQueueComm::sendPacket, this, std::placeholders::_1));

		if (!flightControlSubscription_.connected())
		{
			APLOG_ERROR << "Cannot connect to data_fc_com";
			return true;
		}

		missionControlSubscription_ = ipc->subscribeOnPackets("data_mc_com",
				std::bind(&MessageQueueComm::sendPacket, this, std::placeholders::_1));

		if (!missionControlSubscription_.connected())
		{
			APLOG_ERROR << "Cannot connect to data_mc_com";
			return true;
		}

		tryConnectChannelMixing();
		tryConnectGroundStation();
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
MessageQueueComm::sendPacket(const Packet& packet)
{
	if (!groundStationConnected_)
		return;
	std::lock_guard<std::mutex> lock(senderMutex_);
	groundStationPublisher_.publish(packet);
}

void
MessageQueueComm::receivePacket(const Packet& packet)
{
	auto dp = dataPresentation_.get();
	if (!dp)
	{
		APLOG_ERROR << "Data Presentation missing. Cannot handle receive.";
		return;
	}

	Packet p = packet;
	Target target = dp->extractHeader<Target>(p);

	switch (target)
	{
	case Target::FLIGHT_ANALYSIS:
		flightAnalysisPublisher_.publish(packet);
		break;
	case Target::FLIGHT_CONTROL:
		flightControlPublisher_.publish(packet);
		break;
	case Target::MISSION_CONTROL:
		missionControlPublisher_.publish(packet);
		break;
	case Target::API:
		apiPublisher_.publish(packet);
		break;
	default:
		APLOG_WARN << "Invalid Target: " << static_cast<int>(target);
		break;
	}

}

void
MessageQueueComm::notifyAggregationOnUpdate(const Aggregator& agg)
{
	scheduler_.setFromAggregationIfNotSet(agg);
	dataPresentation_.setFromAggregationIfNotSet(agg);
	ipc_.setFromAggregationIfNotSet(agg);
}

void
MessageQueueComm::tryConnectChannelMixing()
{
	auto ipc = ipc_.get();
	if (!ipc)
	{
		APLOG_ERROR << "IPC missing.";
		return;
	}
	channelMixingSubscription_ = ipc->subscribeOnPackets("data_ch_com",
			std::bind(&MessageQueueComm::sendPacket, this, std::placeholders::_1));

	if (!channelMixingSubscription_.connected())
	{
		auto sched = scheduler_.get();
		sched->schedule(std::bind(&MessageQueueComm::tryConnectChannelMixing, this), Seconds(1));
	}
}

void
MessageQueueComm::tryConnectGroundStation()
{
	auto ipc = ipc_.get();
	if (!ipc)
	{
		APLOG_ERROR << "IPC missing.";
		return;
	}
	groundStationSubscription_ = ipc->subscribeOnPackets("groundstation_autopilot",
			std::bind(&MessageQueueComm::receivePacket, this, std::placeholders::_1));

	if (!groundStationSubscription_.connected())
	{
		groundStationConnected_ = true;
		auto sched = scheduler_.get();
		sched->schedule(std::bind(&MessageQueueComm::tryConnectChannelMixing, this), Seconds(1));
	}
}
