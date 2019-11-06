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
#include <uavAP/Core/DataPresentation/Content.h>
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/MissionControl/LocalFrameManager/LocalFrameManager.h"
#include "uavAP/MissionControl/ManeuverPlanner/ManeuverPlanner.h"
#include "uavAP/MissionControl/MissionPlanner/IMissionPlanner.h"
#include "uavAP/MissionControl/DataHandling/MissionControlDataHandling.h"
#include "uavAP/MissionControl/GlobalPlanner/IGlobalPlanner.h"
#include "uavAP/MissionControl/ConditionManager/ConditionManager.h"
#include "uavAP/MissionControl/WindAnalysis/WindAnalysis.h"
#include "uavAP/MissionControl/Geofencing/Geofencing.h"
#include "uavAP/Core/DataPresentation/DataPresentation.h"
#include "uavAP/Core/IPC/IPC.h"

MissionControlDataHandling::MissionControlDataHandling() :
		lastOverrideSeqNr_(0), period_(Milliseconds(100))
{
}

std::shared_ptr<MissionControlDataHandling>
MissionControlDataHandling::create(const Configuration& configuration)
{
	auto dataHandling = std::make_shared<MissionControlDataHandling>();

	if (!dataHandling->configure(configuration))
	{
		APLOG_ERROR << "DataHandlingIO: Failed to Load Global Configurations";
	}

	return dataHandling;
}

bool
MissionControlDataHandling::configure(const Configuration& configuration)
{
	PropertyMapper<Configuration> propertyMapper(configuration);
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
		publisher_ = ipc->publishPackets(
				EnumMap<Target>::convert(Target::MISSION_CONTROL) + "_to_comm");
		overridePublisher_ = ipc->publishPackets("active_override");
		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = ipc_.get();

		missionControlSubscription_ = ipc->subscribeOnPackets(
				"comm_to_" + EnumMap<Target>::convert(Target::MISSION_CONTROL),
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
	windAnalysis_.setFromAggregationIfNotSet(agg);
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
	Packet trimPacket = dp->serialize(trim);
	dp->addHeader(trimPacket, Content::CONTROLLER_OUTPUT_TRIM);
	publisher_.publish(trimPacket);

	Packet overridePacket = dp->serialize(override);
	dp->addHeader(overridePacket, Content::OVERRIDE);
	publisher_.publish(overridePacket);

	if (mp->getOverrideNr() != lastOverrideSeqNr_)
	{
		lastOverrideSeqNr_ = mp->getOverrideNr();
		overridePublisher_.publish(dp->serialize(override));
	}

	if (auto wa = windAnalysis_.get())
	{
		auto windAnalysisStatus = wa->getWindAnalysisStatus();
		Packet windAnalysisStatusPacket = dp->serialize(windAnalysisStatus);
		dp->addHeader(windAnalysisStatusPacket, Content::WIND_ANALYSIS_STATUS);
		publisher_.publish(windAnalysisStatusPacket);
	}

	//Trajectory hack: Orbit of geofencing
	auto geo = geofencing_.get();
	if (!geo)
	{
		return;
	}

	Packet misPack = dp->serialize(geo->getCriticalPoints());
	dp->addHeader(misPack, Content::CRITICAL_POINTS);
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

	Packet p = packet;
	Content content = dp->extractHeader<Content>(p);

	switch (content)
	{
	case Content::REQUEST_DATA:
	{
		auto request = dp->deserialize<DataRequest>(p);
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
		auto manOverride = dp->deserialize<Override>(p);
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
		auto maneuverSet = dp->deserialize<std::string>(p);
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
		auto mission = dp->deserialize<std::string>(p);
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
		auto frame = dp->deserialize<VehicleOneFrame>(p);
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
		auto offset = dp->deserialize<ControllerOutput>(p);
		auto mp = maneuverPlanner_.get();
		if (!mp)
		{
			APLOG_ERROR << "Maneuver Planner not found. Cannot Set Controller Output Offset.";
			break;
		}
		mp->setControllerOutputOffset(offset);
		break;
	}
	case Content::WIND_ANALYSIS_STATUS:
	{
		auto windAnalysisStatus = dp->deserialize<WindAnalysisStatus>(p);
		auto wa = windAnalysis_.get();
		if (!wa)
		{
			APLOG_ERROR << "Wind Analysis not found. Cannot Set Wind Analysis Status.";
			break;
		}
		wa->setWindAnalysisStatus(windAnalysisStatus);
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
MissionControlDataHandling::collectAndSendMission(std::shared_ptr<DataPresentation> dp)
{
	APLOG_DEBUG << "Collect and send Mission";
	auto gp = globalPlanner_.get();
	if (!gp)
	{
		APLOG_ERROR << "GlobalPlanner missing. Cannot collect and send Mission.";
		return;
	}

	Mission mission = gp->getMission();
	Packet packet = dp->serialize(mission);
	dp->addHeader(packet, Content::MISSION);
	publisher_.publish(packet);
}

void
MissionControlDataHandling::collectAndSendSafetyBounds(std::shared_ptr<DataPresentation> dp)
{
	APLOG_DEBUG << "Collect and Send Safety Bounds.";

	auto maneuverPlanner = maneuverPlanner_.get();

	if (!maneuverPlanner)
	{
		APLOG_ERROR << "Maneuver Planner Missing. Collect and Send Safety Bounds.";
		return;
	}

	auto safetyBounds = maneuverPlanner->getSafetyBounds();
	Packet packet = dp->serialize(safetyBounds);
	dp->addHeader(packet, Content::SAFETY_BOUNDS);
	publisher_.publish(packet);
}

void
MissionControlDataHandling::collectAndSendLocalFrame(std::shared_ptr<DataPresentation> dp)
{
	auto lmf = localFrameManager_.get();
	if (!lmf)
	{
		APLOG_DEBUG << "No local frame manager set. Local frame is earth frame.";
		return;
	}

	auto frame = lmf->getFrame();
	Packet packet = dp->serialize(frame);
	dp->addHeader(packet, Content::LOCAL_FRAME);
	publisher_.publish(packet);
}
