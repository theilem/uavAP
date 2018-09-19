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
 * CustomPlanner.cpp
 *
 *  Created on: Sep 6, 2017
 *      Author: mircot
 */

#include <memory>
#include <mutex>

#include "uavAP/MissionControl/MissionPlanner/CustomPlanner/CustomPlanner.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/Scheduler/IScheduler.h"
#include "uavAP/Core/SensorData.h"
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/MissionControl/GlobalPlanner/IGlobalPlanner.h"

CustomPlanner::CustomPlanner() :
		defaultVelocity_(0), useApproach_(false)
{
}

bool
CustomPlanner::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);

	pm.add<double>("default_velocity", defaultVelocity_, true);
	pm.add<bool>("use_approach", useApproach_, true);
	Mission defaultMission;
	Waypoint centerWP(Vector3(0, 0, 100), defaultVelocity_);
	defaultMission.waypoints.push_back(centerWP);
	missionMap_.insert(std::make_pair("default", defaultMission));

	boost::property_tree::ptree missions;
	pm.add("missions", missions, false);

	for (auto& it : missions)
	{
		Mission mis;
		mis.configure(it.second);
		missionMap_.insert(std::make_pair(it.first, mis));
	}

	currentMission_ = missionMap_.find("default");

	return pm.map();
}

void
CustomPlanner::notifyAggregationOnUpdate(const Aggregator& agg)
{
	ipc_.setFromAggregationIfNotSet(agg);
	scheduler_.setFromAggregationIfNotSet(agg);
	globalPlanner_.setFromAggregationIfNotSet(agg);
}

bool
CustomPlanner::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!ipc_.isSet())
		{
			APLOG_ERROR << "CustomPlanner: IPC Missing.";
			return true;
		}

		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "CustomPlanner: Scheduler Missing.";
			return true;
		}

		if (!globalPlanner_.isSet())
		{
			APLOG_ERROR << "CustomPlanner: Global Planner Missing.";
			return true;
		}

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = ipc_.get();

		sensorDataSubscription_ = ipc->subscribeOnSharedMemory<SensorData>("sensor_data",
				std::bind(&CustomPlanner::onSensorData, this, std::placeholders::_1));

		if (!sensorDataSubscription_.connected())
		{
			APLOG_ERROR << "Sensor Data Missing.";
			return true;
		}

		auto scheduler = scheduler_.get();
		scheduler->schedule(std::bind(&CustomPlanner::publishMission, this), Milliseconds(0));
		break;
	}
	case RunStage::FINAL:
	{
		break;
	}
	default:
		break;
	}
	return false;
}

void
CustomPlanner::missionRequest(const std::string& mission)
{
	currentMission_ = missionMap_.find(mission);

	if (currentMission_ == missionMap_.end())
	{
		APLOG_ERROR << "Requested Mission " << mission << " Not Found.";
		return;
	}

	publishMission();
}

void
CustomPlanner::publishMission()
{
	if (currentMission_ == missionMap_.end())
	{
		APLOG_ERROR << "No Mission Selected. Cannot Publish.";
		return;
	}

	auto gp = globalPlanner_.get();

	if (!gp)
	{
		APLOG_ERROR << "Cannot Set Mission. Global Planner Missing.";
		return;
	}

	APLOG_DEBUG << "Start Mission: " << currentMission_->first;

	Mission mission = currentMission_->second;

	if (useApproach_)
	{
		std::unique_lock<std::mutex> lock(positionMutex_);
		mission.initialPosition = Waypoint(currentPosition_, currentDirection_);
	}

	gp->setMission(mission);
}

void
CustomPlanner::onSensorData(const SensorData& data)
{
	std::unique_lock<std::mutex> lock_position(positionMutex_);
	currentPosition_ = data.position;
	currentDirection_ = directionFromAttitude(data.attitude);
	lock_position.unlock();
}
