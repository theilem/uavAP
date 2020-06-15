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
#include <cpsCore/Utilities/LockTypes.hpp>
#include <cpsCore/Utilities/Scheduler/IScheduler.h>

#include "uavAP/MissionControl/MissionPlanner/CustomPlanner/CustomPlanner.h"
#include <cpsCore/Utilities/DataPresentation/detail/BasicSerialization.h>
#include "uavAP/Core/SensorData.h"
#include "uavAP/MissionControl/GlobalPlanner/IGlobalPlanner.h"
#include <cpsCore/Utilities/IPC/IPC.h>

CustomPlanner::CustomPlanner() :
		defaultVelocity_(0), defaultAltitude_(100.0), useApproach_(false)
{
}

bool
CustomPlanner::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);

	pm.add<double>("default_velocity", defaultVelocity_, true);
	pm.add<double>("default_altitude", defaultAltitude_, true);
	pm.add<bool>("use_approach", useApproach_, true);
	Mission defaultMission;
	Waypoint centerWP(Vector3(0, 0, defaultAltitude_), defaultVelocity_);
	defaultMission.waypoints.push_back(centerWP);
	missionMap_.insert(std::make_pair("default", defaultMission));

	Configuration missions;
	pm.add("missions", missions, false);

	for (auto& it : missions)
	{
		Mission mis;
		mis.configure(it.second);
		missionMap_.insert(std::make_pair(it.first, mis));
	}

	std::string defMission;
	if (pm.add("default_mission", defMission, false))
	{
		currentMission_ = missionMap_.find(defMission);
	}
	else
	{
		currentMission_ = missionMap_.find("default");
	}

	return pm.map();
}

bool
CustomPlanner::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!checkIsSet<IPC, IScheduler, IGlobalPlanner>())
		{
			CPSLOG_ERROR << "CustomPlanner: Missing deps.";
			return true;
		}

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = get<IPC>();

		sensorDataSubscription_ = ipc->subscribe<SensorData>("sensor_data",
				std::bind(&CustomPlanner::onSensorData, this, std::placeholders::_1));

		if (!sensorDataSubscription_.connected())
		{
			CPSLOG_ERROR << "Sensor Data Missing.";
			return true;
		}

		auto scheduler = get<IScheduler>();
		scheduler->schedule(std::bind(&CustomPlanner::publishMission, this), Milliseconds(100));
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
		CPSLOG_ERROR << "Requested Mission " << mission << " Not Found.";
		return;
	}

	publishMission();
}

void
CustomPlanner::publishMission()
{
	if (currentMission_ == missionMap_.end())
	{
		CPSLOG_ERROR << "No Mission Selected. Cannot Publish.";
		return;
	}

	auto gp = get<IGlobalPlanner>();

	if (!gp)
	{
		CPSLOG_ERROR << "Cannot Set Mission. Global Planner Missing.";
		return;
	}

	CPSLOG_DEBUG << "Start Mission: " << currentMission_->first;

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
	LockGuard lockPosition(positionMutex_);
	currentPosition_ = data.position;
	currentDirection_ = directionFromAttitude(data.attitude);
}
