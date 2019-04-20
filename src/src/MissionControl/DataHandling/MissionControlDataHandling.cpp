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
/*
 * MissionControlDataHandling.cpp
 *
 *  Created on: Sep 6, 2017
 *      Author: mircot
 */
#include "uavAP/Core/DataPresentation/ContentMapping.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/MissionControl/LocalFrameManager/LocalFrameManager.h"
#include "uavAP/MissionControl/ManeuverPlanner/ManeuverPlanner.h"
#include "uavAP/MissionControl/MissionPlanner/IMissionPlanner.h"
#include "uavAP/MissionControl/DataHandling/MissionControlDataHandling.h"
#include "uavAP/MissionControl/GlobalPlanner/IGlobalPlanner.h"
#include "uavAP/MissionControl/ConditionManager/ConditionManager.h"
#include "uavAP/MissionControl/Geofencing/Geofencing.h"
#include "uavAP/Core/DataPresentation/IDataPresentation.h"
#include "uavAP/Core/DataPresentation/BinarySerialization.hpp"

MissionControlDataHandling::MissionControlDataHandling() :
		lastOverrideSeqNr_(0), period_(Milliseconds(100))
{
}

std::shared_ptr<MissionControlDataHandling>
MissionControlDataHandling::create(const boost::property_tree::ptree& configuration)
{
	auto dataHandling = std::make_shared<MissionControlDataHandling>();

	if (!dataHandling->configure(configuration))
	{
		APLOG_ERROR << "DataHandlingIO: Failed to Load Global Configurations";
	}

	return dataHandling;
}

bool
MissionControlDataHandling::configure(const boost::property_tree::ptree& configuration)
{
	PropertyMapper propertyMapper(configuration);
	propertyMapper.add("period", period_, false);

	return propertyMapper.map();
}

bool
MissionControlDataHandling::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "DataHandling: Scheduler missing.";

			return true;
		}
		if (!ipc_.isSet())
		{
			APLOG_ERROR << "DataHandling: IPC missing.";

			return true;
		}
		if (!dataPresentation_.isSet())
		{
			APLOG_ERROR << "DataHandling: DataPresentation missing.";

			return true;
		}
		if (!globalPlanner_.isSet())
		{
			APLOG_WARN << "DataHandling: global planner missing.";
		}
		if (!conditionManager_.isSet())
		{
			APLOG_WARN << "DataHandling: Condition Manager Missing.";
		}

		auto ipc = ipc_.get();
		publisher_ = ipc->publishPackets("data_mc_com");
		overridePublisher_ = ipc->publishPackets("active_override");
		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = ipc_.get();

		missionControlSubscription_ = ipc->subscribeOnPacket("data_com_mc",
				boost::bind(&MissionControlDataHandling::receiveAndDistribute, this, _1));

		if (!missionControlSubscription_.connected())
		{
			APLOG_ERROR << "DataHandlingIO: Failed to Subscribe On Flight Control Message Queue.";

			return true;
		}

		auto scheduler = scheduler_.get();
		scheduler->schedule(std::bind(&MissionControlDataHandling::collectAndSend, this),
				Milliseconds(0), period_);

		break;
	}
	case RunStage::FINAL:
	{
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
MissionControlDataHandling::notifyAggregationOnUpdate(const Aggregator& agg)
{
	globalPlanner_.setFromAggregationIfNotSet(agg);
	scheduler_.setFromAggregationIfNotSet(agg);
	dataPresentation_.setFromAggregationIfNotSet(agg);
	ipc_.setFromAggregationIfNotSet(agg);
	maneuverPlanner_.setFromAggregationIfNotSet(agg);
	missionPlanner_.setFromAggregationIfNotSet(agg);
	localFrameManager_.setFromAggregationIfNotSet(agg);
	conditionManager_.setFromAggregationIfNotSet(agg);
	geofencing_.setFromAggregationIfNotSet(agg);
}

void
MissionControlDataHandling::collectAndSend()
{
	auto dp = dataPresentation_.get();
	if (!dp)
	{
		APLOG_ERROR << "Data presentation missing. Cannot collect and send.";
		return;
	}

	auto mp = maneuverPlanner_.get();
	auto override = mp->getOverride();
	auto trim = mp->getControllerOutputTrim();
	Packet overridePacket = dp->serialize(override, Content::OVERRIDE);
	Packet trimPacket = dp->serialize(trim, Content::CONTROLLER_OUTPUT_TRIM);
	publisher_.publish(overridePacket);
	publisher_.publish(trimPacket);

	if (mp->getOverrideNr() != lastOverrideSeqNr_)
	{
		lastOverrideSeqNr_ = mp->getOverrideNr();
		overridePublisher_.publish(dp::serialize(override));
	}

	//Trajectory hack: Orbit of geofencing
	auto geo = geofencing_.get();
	if (!geo)
	{
		return;
	}

	Packet misPack = dp->serialize(geo->criticalPoints(), Content::MISSION);
	publisher_.publish(misPack);

}

void
MissionControlDataHandling::receiveAndDistribute(const Packet& packet)
{
	auto dp = dataPresentation_.get();

	if (!dp)
	{
		APLOG_ERROR << "Data presentaiton missing. Cannot Handle packet.";
		return;
	}

	Content content = Content::INVALID;
	auto any = dp->deserialize(packet, content);

	switch (content)
	{
	case Content::REQUEST_DATA:
	{
		auto request = boost::any_cast<DataRequest>(any);
		switch (request)
		{
		case DataRequest::MISSION:
			collectAndSendMission(dp);
			break;
		case DataRequest::SAFETY_BOUNDS:
			collectAndSendSafetyBounds(dp);
			break;
		case DataRequest::LOCAL_FRAME:
			collectAndSendLocalFrame(dp);
			break;
		default:
			APLOG_WARN << "Received invalid data request: " << static_cast<int>(request);
			break;
		}

		break;
	}
	case Content::OVERRIDE:
	{
		auto manOverride = boost::any_cast<Override>(any);
		auto mp = maneuverPlanner_.get();
		if (!mp)
		{
			APLOG_ERROR << "Maneuver Planner not found. Cannot override.";
			break;
		}
		mp->setManualOverride(manOverride);
		break;
	}
	case Content::SELECT_MANEUVER_SET:
	{
		auto maneuverSet = boost::any_cast<std::string>(any);
		auto mp = maneuverPlanner_.get();
		if (!mp)
		{
			APLOG_ERROR << "Maneuver Planner not found. Cannot override.";
			break;
		}
		mp->setManeuverOverride(maneuverSet);
		break;
	}
	case Content::SELECT_MISSION:
	{
		auto mission = boost::any_cast<std::string>(any);
		auto mp = missionPlanner_.get();
		if (!mp)
		{
			APLOG_ERROR << "Mission Planner not found. Cannot override.";
			break;
		}
		mp->missionRequest(mission);
		break;
	}
	case Content::LOCAL_FRAME:
	{
		auto frame = boost::any_cast<VehicleOneFrame>(any);
		auto lfm = localFrameManager_.get();
		if (!lfm)
		{
			APLOG_ERROR << "Maneuver Planner Not Found. Cannot Set Local Frame.";
			break;
		}
		lfm->setFrame(frame);
		break;
	}
	case Content::CONTROLLER_OUTPUT_OFFSET:
	{
		auto offset = boost::any_cast<ControllerOutput>(any);
		auto mp = maneuverPlanner_.get();
		if (!mp)
		{
			APLOG_ERROR << "Maneuver Planner not found. Cannot Set Controller Output Offset.";
			break;
		}
		mp->setControllerOutputOffset(offset);
		break;
	}
	default:
	{
		APLOG_ERROR << "Unspecified Content: " << static_cast<int>(content);
		break;
	}
	}
}

void
MissionControlDataHandling::collectAndSendMission(
		std::shared_ptr<IDataPresentation<Content, Target>> dp)
{
	APLOG_DEBUG << "Collect and send Mission";
	auto gp = globalPlanner_.get();
	if (!gp)
	{
		APLOG_ERROR << "GlobalPlanner missing. Cannot collect and send Mission.";
		return;
	}

	Mission mission = gp->getMission();
	Packet packet = dp->serialize(mission, Content::MISSION);
	publisher_.publish(packet);
}

void
MissionControlDataHandling::collectAndSendSafetyBounds(
		std::shared_ptr<IDataPresentation<Content, Target>> dp)
{
	APLOG_DEBUG << "Collect and Send Safety Bounds.";

	auto maneuverPlanner = maneuverPlanner_.get();

	if (!maneuverPlanner)
	{
		APLOG_ERROR << "Maneuver Planner Missing. Collect and Send Safety Bounds.";
		return;
	}

	auto safetyBounds = maneuverPlanner->getSafetyBounds();
	Packet packet = dp->serialize(safetyBounds, Content::SAFETY_BOUNDS);
	publisher_.publish(packet);
}

void
MissionControlDataHandling::collectAndSendLocalFrame(
		std::shared_ptr<IDataPresentation<Content, Target> > dp)
{
	auto lmf = localFrameManager_.get();
	if (!lmf)
	{
		APLOG_DEBUG << "No local frame manager set. Local frame is earth frame.";
		return;
	}

	auto frame = lmf->getFrame();
	Packet packet = dp->serialize(frame, Content::LOCAL_FRAME);
	publisher_.publish(packet);
}
