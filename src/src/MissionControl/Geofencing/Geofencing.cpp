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
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>
#include <uavAP/Core/Scheduler/IScheduler.h>
#include <uavAP/Core/SensorData.h>
#include <uavAP/MissionControl/Geofencing/Geofencing.h>
#include <uavAP/MissionControl/ManeuverPlanner/ManeuverPlanner.h>
#include <uavAP/Core/IPC/IPC.h>
#include <uavAP/Core/Object/AggregatableObjectImpl.hpp>
#include <uavAP/MissionControl/WindAnalysis/WindAnalysis.h>

Geofencing::Geofencing() :
		leftSafe_(true), rightSafe_(true), safetyActiveLeft_(false), safetyActiveRight_(false)
{
}

bool
Geofencing::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!checkIsSet<IPC, IScheduler, ManeuverPlanner, IGeofencingModel>())
		{
			APLOG_ERROR << "Geofencing: Missing dependencies.";
			return true;
		}
		if (!isSet<WindAnalysis>())
		{
			APLOG_WARN << "No wind analysis. Geofencing not good.";
		}
		windEstimate_.velocity = params.windEstimate();
		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = get<IPC>();

		ipc->subscribe<SensorData>("sensor_data",
				std::bind(&Geofencing::onSensorData, this, std::placeholders::_1));

		auto mp = get<ManeuverPlanner>();

		geoFence_.fromRectangle(mp->getSafetyBounds());

		auto scheduler = get<IScheduler>();

		scheduler->schedule(std::bind(&Geofencing::evaluateSafety, this),
				Milliseconds(params.period()), Milliseconds(params.period()));

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

std::vector<Waypoint>
Geofencing::getCriticalPoints() const
{
	std::vector<Waypoint> criticalPoints;
//
//	auto geofencingModel = get<IGeofencingModel>();
//
//	if (!geofencingModel)
//	{
//		APLOG_ERROR << "Geofencing: Geofencing Model Missing.";
//		return std::vector<Waypoint>();
//	}
//
//	for (const auto& it : geoFence_.getEdges())
//	{
//		if (it.getDistanceAbs(sensorData_.position.head(2)) > params.evaluationThreshold())
//			continue;
//		for (const auto& points : geofencingModel->getCriticalPoints(it,
//				IGeofencingModel::RollDirection::LEFT))
//		{
//			criticalPoints.push_back(Waypoint(points));
//		}
//		for (const auto& points : geofencingModel->getCriticalPoints(it,
//				IGeofencingModel::RollDirection::RIGHT))
//		{
//			criticalPoints.push_back(Waypoint(points));
//		}
//	}
	LockGuard l(criticalPointsMutex_);
	criticalPoints.insert(criticalPoints.end(), criticalPointsLeft_.begin(),
			criticalPointsLeft_.end());
	criticalPoints.insert(criticalPoints.end(), criticalPointsRight_.begin(),
			criticalPointsRight_.end());

	return criticalPoints;
}

void
Geofencing::onSensorData(const SensorData& data)
{
	Lock lock(sensorDataMutex_, std::try_to_lock);
	if (!lock.owns_lock())
		return;
	sensorData_ = data;
}

void
Geofencing::evaluateSafety()
{
	auto geofencingModel = get<IGeofencingModel>();

	if (!geofencingModel)
	{
		APLOG_ERROR << "Geofencing: Geofencing Model Missing.";
		return;
	}

	if (auto wa = get<WindAnalysis>())
	{
		windEstimate_ = wa->getWindInfo();
	}

	Lock lock(sensorDataMutex_);
	geofencingModel->updateModel(sensorData_, windEstimate_);
	Vector3 position = sensorData_.position;
	lock.unlock();

	auto mp = get<ManeuverPlanner>();

	if (!mp)
	{
		APLOG_ERROR << "Geofencing::evaluateSafety(): Manevuer planner missing";
		return;
	}

	bool leftSafe = true, rightSafe = true;
	Lock l(criticalPointsMutex_);
	criticalPointsLeft_.clear();
	criticalPointsRight_.clear();
	for (const auto& it : geoFence_.getEdges())
	{
		if (it.getDistanceAbs(position.head(2)) > params.evaluationThreshold())
			continue;
		for (const auto& points : geofencingModel->getCriticalPoints(it,
				IGeofencingModel::RollDirection::LEFT))
		{
			criticalPointsLeft_.push_back(Waypoint(points));
			if (it.getDistance(points.head(2)) < params.distanceThreshold())
			{
				APLOG_WARN << "Left not safe";
				leftSafe = false;
				break;
			}
		}
		for (const auto& points : geofencingModel->getCriticalPoints(it,
				IGeofencingModel::RollDirection::RIGHT))
		{
			criticalPointsRight_.push_back(Waypoint(points));
			if (it.getDistance(points.head(2)) < params.distanceThreshold())
			{
				APLOG_WARN << "Right not safe";
				rightSafe = false;
				break;
			}
		}
	}
	l.unlock();

	if (!leftSafe && !rightSafe)
	{
		if (safetyActiveLeft_ || safetyActiveRight_)
			return;
		Override override;
		if (leftSafe_)
		{
			APLOG_WARN << "Override left initiated";
			override.pid.insert(std::make_pair(PIDs::ROLL, -params.rollMax()));
			safetyActiveLeft_ = true;
		}
		else if (rightSafe_)
		{
			APLOG_WARN << "Override right initiated";
			override.pid.insert(std::make_pair(PIDs::ROLL, params.rollMax()));
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
