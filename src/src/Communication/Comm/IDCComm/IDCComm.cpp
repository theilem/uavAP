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
#include "uavAP/Core/DataPresentation/DataPresentation.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/FlightControl/Controller/IController.h"
#include "uavAP/Core/Scheduler/IScheduler.h"
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/LockTypes.h"

IDCComm::IDCComm()
{
}

std::shared_ptr<IDCComm>
IDCComm::create(const Configuration& configuration)
{
	auto serialComm = std::make_shared<IDCComm>();

	if (!serialComm->configure(configuration))
	{
		APLOG_ERROR << "SerialComm: Failed to Load Global Configurations";
	}

	return serialComm;
}

bool
IDCComm::configure(const Configuration& configuration)
{
	PropertyMapper pm(configuration);

	return pm.map();
}

bool
IDCComm::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "SerialComm: Scheduler missing.";
			return true;
		}
		if (!ipc_.isSet())
		{
			APLOG_ERROR << "SerialComm: IPC missing";
			return true;
		}

		auto ipc = ipc_.get();
		flightAnalysisPublisher_ = ipc->publishPackets("data_com_fa");
		flightControlPublisher_ = ipc->publishPackets("comm_to_flight_control");
		missionControlPublisher_ = ipc->publishPackets("data_com_mc");
		apiPublisher_ = ipc->publishPackets("data_com_api");

		if (!idc_.isSet())
		{
			APLOG_ERROR << "SerialComm: IDC missing.";
			return true;
		}
		auto idc = idc_.get();
		sender_ = idc->createSender("ground_station");

		idcConnection_ = idc->subscribeOnPacket("ground_station",
				std::bind(&IDCComm::receivePacket, this, std::placeholders::_1));

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = ipc_.get();

		flightAnalysisSubscription_ = ipc->subscribeOnPackets("data_fa_com",
				std::bind(&IDCComm::sendPacket, this, std::placeholders::_1));

		if (!flightAnalysisSubscription_.connected())
		{
			APLOG_WARN << "Cannot connect to data_fa_com. Ignoring.";
		}

		flightControlSubscription_ = ipc->subscribeOnPackets("flight_control_to_comm",
				std::bind(&IDCComm::sendPacket, this, std::placeholders::_1));

		if (!flightControlSubscription_.connected())
		{
			APLOG_ERROR << "Cannot connect to data_fc_com";
			return true;
		}

		missionControlSubscription_ = ipc->subscribeOnPackets("data_mc_com",
				std::bind(&IDCComm::sendPacket, this, std::placeholders::_1));

		if (!missionControlSubscription_.connected())
		{
			APLOG_ERROR << "Cannot connect to data_mc_com";
			return true;
		}

		tryConnectChannelMixing();
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
	std::lock_guard<std::mutex> lock(senderMutex_);
	sender_.sendPacket(packet);
}

void
IDCComm::receivePacket(const Packet& packet)
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
IDCComm::notifyAggregationOnUpdate(const Aggregator& agg)
{
	scheduler_.setFromAggregationIfNotSet(agg);
	dataPresentation_.setFromAggregationIfNotSet(agg);
	ipc_.setFromAggregationIfNotSet(agg);
	idc_.setFromAggregationIfNotSet(agg);
}

void
IDCComm::tryConnectChannelMixing()
{
	auto ipc = ipc_.get();
	if (!ipc)
	{
		APLOG_ERROR << "IPC missing.";
		return;
	}
	channelMixingSubscription_ = ipc->subscribeOnPackets("data_ch_com",
			std::bind(&IDCComm::sendPacket, this, std::placeholders::_1));

	if (!channelMixingSubscription_.connected())
	{
		auto sched = scheduler_.get();
		sched->schedule(std::bind(&IDCComm::tryConnectChannelMixing, this), Seconds(1));
	}
}
