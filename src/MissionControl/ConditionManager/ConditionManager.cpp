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
#include "uavAP/MissionControl/ConditionManager/ConditionManager.h"
#include "uavAP/MissionControl/ConditionManager/Condition/RectanguloidCondition.h"
#include "cpsCore/Utilities/IPC/IPC.h"
#include "cpsCore/Utilities/DataPresentation/DataPresentation.h"

std::shared_ptr<ConditionManager>
ConditionManager::create(const Configuration& config)
{
	auto conditionManager = std::make_shared<ConditionManager>();

	if (!conditionManager->configure(config))
	{
		CPSLOG_ERROR << "ConditionManager: Failed to Load Config.";
	}

	return conditionManager;
}

bool
ConditionManager::configure(const Configuration& config)
{
	return true;
}

bool
ConditionManager::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!checkIsSet<IPC, IScheduler, DataPresentation>())
		{
			CPSLOG_ERROR << "ManeuverAnalysis: Missing dependencies.";
			return true;
		}

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = get<IPC>();

		sensorDataSubscription_ = ipc->subscribe<SensorData>("sensor_data",
				std::bind(&ConditionManager::onSensorData, this, std::placeholders::_1));

		if (!sensorDataSubscription_.connected())
		{
			CPSLOG_ERROR << "ConditionManager: Sensor Data Missing.";

			return true;
		}

		steadyStateSubscription_ = ipc->subscribeOnPackets("steady_state",
				std::bind(&ConditionManager::onSteadyState, this, std::placeholders::_1));

		if (!steadyStateSubscription_.connected())
		{
			CPSLOG_ERROR << "ConditionManager: Steady State Missing.";

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
	return get<IScheduler>();
}

std::weak_ptr<DataPresentation>
ConditionManager::getDataPresentation() const
{
	return get<DataPresentation>();
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
