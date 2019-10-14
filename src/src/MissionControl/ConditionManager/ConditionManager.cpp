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
 * ConditionManager.cpp
 *
 *  Created on: Jul 29, 2018
 *      Author: simonyu
 */

#include "uavAP/Core/Frames/InertialFrame.h"
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/MissionControl/ConditionManager/ConditionManager.h"
#include "uavAP/MissionControl/ConditionManager/Condition/RectanguloidCondition.h"
#include "uavAP/Core/IPC/IPC.h"

std::shared_ptr<ConditionManager>
ConditionManager::create(const Configuration& config)
{
	auto conditionManager = std::make_shared<ConditionManager>();

	if (!conditionManager->configure(config))
	{
		APLOG_ERROR << "ConditionManager: Failed to Load Config.";
	}

	return conditionManager;
}

bool
ConditionManager::configure(const Configuration& config)
{
	return true;
}

void
ConditionManager::notifyAggregationOnUpdate(const Aggregator& agg)
{
	ipcHandle_.setFromAggregationIfNotSet(agg);
	schedulerHandle_.setFromAggregationIfNotSet(agg);
}

bool
ConditionManager::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!ipcHandle_.isSet())
		{
			APLOG_ERROR << "ConditionManager: IPC Missing.";
			return true;
		}

		if (!schedulerHandle_.isSet())
		{
			APLOG_ERROR << "ConditionManager: Scheduler Missing.";
			return true;
		}

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = ipcHandle_.get();

		sensorDataSubscription_ = ipc->subscribe<SensorData>("sensor_data",
				std::bind(&ConditionManager::onSensorData, this, std::placeholders::_1));

		if (!sensorDataSubscription_.connected())
		{
			APLOG_ERROR << "ConditionManager: Sensor Data Missing.";

			return true;
		}

		steadyStateSubscription_ = ipc->subscribeOnPackets("steady_state",
				std::bind(&ConditionManager::onSteadyState, this, std::placeholders::_1));

		if (!steadyStateSubscription_.connected())
		{
			APLOG_ERROR << "ConditionManager: Steady State Missing.";

//			return true;
		}

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

boost::signals2::connection
ConditionManager::subscribeOnSensorData(const OnSensorData::slot_type& slot)
{
	return onSensorData_.connect(slot);
}

boost::signals2::connection
ConditionManager::subscribeOnSteadyState(const OnSteadyState::slot_type& slot)
{
	return onSteadyState_.connect(slot);
}

void
ConditionManager::activateCondition(std::shared_ptr<ICondition> condition, const ICondition::ConditionTrigger& conditionTrigger)
{
	condition->activate(this, conditionTrigger);
}

void
ConditionManager::deactivateCondition(std::shared_ptr<ICondition> condition)
{
	condition->deactivate();
}

std::weak_ptr<IScheduler>
ConditionManager::getScheduler() const
{
	return schedulerHandle_.get();
}

void
ConditionManager::onSensorData(const SensorData& sd)
{
	onSensorData_(sd);
}

void
ConditionManager::onSteadyState(const Packet& packet)
{
	onSteadyState_(packet);
}
