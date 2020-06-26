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
#include "uavAP/Core/DataHandling/DataHandling.h"
#include "uavAP/MissionControl/GlobalPlanner/IGlobalPlanner.h"
#include <cpsCore/Utilities/IPC/IPC.h>

CustomPlanner::CustomPlanner() :
		defaultVelocity_(0), defaultAltitude_(100.0), useApproach_(false)
{
}

//bool
//CustomPlanner::configure(const Configuration& config)
//{
//	PropertyMapper<Configuration> pm(config);
//	CPSLOG_ERROR << "Mission configure called";
//
//	pm.add<double>("default_velocity", defaultVelocity_, true);
//	pm.add<double>("default_altitude", defaultAltitude_, true);
//	pm.add<bool>("use_approach", useApproach_, true);
//	Mission defaultMission;
//	Waypoint centerWP(Vector3(0, 0, defaultAltitude_), defaultVelocity_);
//	defaultMission.waypoints.push_back(centerWP);
//	missionMap_.insert(std::make_pair("default", defaultMission));
//
//	Configuration missions;
//	pm.add("missions", missions, false);
//
//	for (auto& it : missions)
//	{
//		Mission mis;
//		mis.configure(it.second);
//		missionMap_.insert(std::make_pair(it.first, mis));
//	}
//
//	std::string defMission;
//	if (pm.add("default_mission", defMission, false))
//	{
//		currentMission_ = missionMap_.find(defMission);
//	}
//	else
//	{
//		currentMission_ = missionMap_.find("default");
//	}
//
//	return pm.map();
//}

bool
CustomPlanner::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			if (!checkIsSet<IPC, IScheduler, IGlobalPlanner, DataHandling>())
			{
				CPSLOG_ERROR << "CustomPlanner: Missing deps.";
				return true;
			}

			//Creating default mission
			Waypoint centerWP;
			centerWP.location = Vector3(0,0, params.defaultAltitude());
			Mission defaultMission;
			defaultMission.velocity = params.defaultVelocity();
			defaultMission.infinite = true;
			defaultMission.waypoints = {centerWP};

			auto it = params.missions().insert(std::make_pair("default_mission", defaultMission));
			if (it.second)
				currentMission_ = it.first;

			break;
		}
		case RunStage::NORMAL:
		{
			auto ipc = get<IPC>();

			sensorDataSubscription_ = ipc->subscribe<SensorData>("sensor_data",
																 std::bind(&CustomPlanner::onSensorData, this,
																		   std::placeholders::_1));

			if (!sensorDataSubscription_.connected())
			{
				CPSLOG_ERROR << "Sensor Data Missing.";
				return true;
			}

			if (auto dh = get<DataHandling>())
			{
				dh->subscribeOnData<std::string>(Content::SELECT_MISSION,
													std::bind(&CustomPlanner::missionRequest, this,
															  std::placeholders::_1));
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
	auto newMission = missionMap_.find(mission);

	if (newMission == missionMap_.end())
	{
		CPSLOG_ERROR << "Requested Mission " << mission << " Not Found.";
		return;
	}

	currentMission_ = newMission;

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
		Waypoint wp;
		wp.location = currentPosition_;
		wp.direction = currentDirection_;
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
