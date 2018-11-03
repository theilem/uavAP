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
 * Geofencing.cpp
 *
 *  Created on: Aug 20, 2018
 *      Author: mircot
 */
#include <uavAP/Core/IPC/IPC.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>
#include <uavAP/Core/Scheduler/IScheduler.h>
#include <uavAP/Core/SensorData.h>
#include <uavAP/MissionControl/Geofencing/Geofencing.h>
#include <uavAP/MissionControl/ManeuverPlanner/ManeuverPlanner.h>

Geofencing::Geofencing() :
		leftSafe_(true), rightSafe_(true), safetyActiveLeft_(false), safetyActiveRight_(false), rollMax_(
				0), evaluationThreshold_(
		DBL_MAX), distanceThreshold_(0)
{
}

bool
Geofencing::configure(const Configuration& config)
{
	PropertyMapper pm(config);
	pm.add<double>("roll_max", rollMax_, true);
	pm.add<double>("evaluation_threshold", evaluationThreshold_, true);
	pm.add<double>("distance_threshold", distanceThreshold_, true);
	pm.add("period", period_, true);

	degToRadRef(rollMax_);

	return pm.map();
}

void
Geofencing::notifyAggregationOnUpdate(const Aggregator& agg)
{
	ipc_.setFromAggregationIfNotSet(agg);
	scheduler_.setFromAggregationIfNotSet(agg);
	maneuverPlanner_.setFromAggregationIfNotSet(agg);
	geofencingModel_.setFromAggregationIfNotSet(agg);
}

bool
Geofencing::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!ipc_.isSet())
		{
			APLOG_ERROR << "Geofencing: ipc not set";
			return true;
		}
		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "Geofencing: Scheduler not set";
			return true;
		}
		if (!maneuverPlanner_.isSet())
		{
			APLOG_ERROR << "Geofencing: ManeuverPlanner not set";
			return true;
		}
		if (!geofencingModel_.isSet())
		{
			APLOG_ERROR << "Geofencing: Geofencing Model Missing.";
			return true;
		}
		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = ipc_.get();

		ipc->subscribeOnSharedMemory<SensorData>("sensor_data",
				std::bind(&Geofencing::onSensorData, this, std::placeholders::_1));

		auto mp = maneuverPlanner_.get();

		geoFence_.fromRectangle(mp->getSafetyBounds());

		auto scheduler = scheduler_.get();

		scheduler->schedule(std::bind(&Geofencing::evaluateSafety, this), period_, period_);

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

Mission
Geofencing::criticalPoints()
{
	Mission mission;

	auto geofencingModel = geofencingModel_.get();

	if (!geofencingModel)
	{
		APLOG_ERROR << "Geofencing: Geofencing Model Missing.";
		return Mission();
	}

	for (const auto& it : geoFence_.getEdges())
	{
		if (it.getDistanceAbs(sensorData_.position.head(2)) > evaluationThreshold_)
			continue;
		for (const auto& points : geofencingModel->getCriticalPoints(it,
				IGeofencingModel::RollDirection::LEFT))
		{
			mission.waypoints.push_back(Waypoint(points));
		}
		for (const auto& points : geofencingModel->getCriticalPoints(it,
				IGeofencingModel::RollDirection::RIGHT))
		{
			mission.waypoints.push_back(Waypoint(points));
		}
	}

	return mission;
}

void
Geofencing::onSensorData(const SensorData& data)
{
	std::unique_lock<std::mutex> lock(sensorDataMutex_, std::try_to_lock);
	if (!lock.owns_lock())
		return;
	sensorData_ = data;
}

void
Geofencing::evaluateSafety()
{
  	auto geofencingModel = geofencingModel_.get();

	if (!geofencingModel)
	{
		APLOG_ERROR << "Geofencing: Geofencing Model Missing.";
		return;
	}

	std::unique_lock<std::mutex> lock(sensorDataMutex_);
	geofencingModel->updateModel(sensorData_);
	Vector3 position = sensorData_.position;
	lock.unlock();

	auto mp = maneuverPlanner_.get();

	if (!mp)
	{
		APLOG_ERROR << "Geofencing::evaluateSafety(): Manevuer planner missing";
		return;
	}

	bool leftSafe = true, rightSafe = true;
	for (const auto& it : geoFence_.getEdges())
	{
		if (it.getDistanceAbs(position.head(2)) > evaluationThreshold_)
			continue;
		for (const auto& points : geofencingModel->getCriticalPoints(it,
				IGeofencingModel::RollDirection::LEFT))
		{
			if (it.getDistance(points.head(2)) < distanceThreshold_)
			{
				APLOG_WARN << "Left not safe";
				leftSafe = false;
				break;
			}
		}
		for (const auto& points : geofencingModel->getCriticalPoints(it,
				IGeofencingModel::RollDirection::RIGHT))
		{
			if (it.getDistance(points.head(2)) < distanceThreshold_)
			{
				APLOG_WARN << "Right not safe";
				rightSafe = false;
				break;
			}
		}
	}

	if (!leftSafe && !rightSafe)
	{
		if (safetyActiveLeft_ || safetyActiveRight_)
			return;
		Override override;
		if (leftSafe_)
		{
			APLOG_WARN << "Override left initiated";
			override.pid.insert(std::make_pair(PIDs::ROLL, -rollMax_));
			safetyActiveLeft_ = true;
		}
		else if (rightSafe_)
		{
			APLOG_WARN << "Override right initiated";
			override.pid.insert(std::make_pair(PIDs::ROLL, rollMax_));
			safetyActiveRight_ = true;
		}
		else
		{
			APLOG_ERROR << "That should not have happened";
		}
		mp->setManualOverride(override);
		return;
	}

	if ((leftSafe && safetyActiveRight_) || (rightSafe && safetyActiveLeft_))
	{
		Override override;
		mp->setManualOverride(override);
		safetyActiveLeft_ = false;
		safetyActiveRight_ = false;
	}

	leftSafe_ = leftSafe;
	rightSafe_ = rightSafe;
}
