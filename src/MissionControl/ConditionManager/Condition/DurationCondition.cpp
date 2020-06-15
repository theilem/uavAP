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
 * DurationCondition.cpp
 *
 *  Created on: Jul 29, 2018
 *      Author: simonyu
 */

#include <cpsCore/Utilities/Scheduler/IScheduler.h>
#include "uavAP/MissionControl/ConditionManager/ConditionManager.h"
#include "uavAP/MissionControl/ConditionManager/Condition/DurationCondition.h"

std::shared_ptr<DurationCondition>
DurationCondition::create(const Configuration& config)
{
	auto durationCondition = std::make_shared<DurationCondition>();

	if (!durationCondition->configure(config))
	{
		CPSLOG_ERROR << "DurationCondition: Failed to Load Config.";
	}

	return durationCondition;
}

bool
DurationCondition::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);

	pm.add("duration", duration_, true);

	return pm.map();
}

void
DurationCondition::activate(ConditionManager* conditionManager,
		const ConditionTrigger& conditionTrigger)
{
	auto scheduler = conditionManager->getScheduler().lock();

	if (!scheduler)
	{
		CPSLOG_ERROR << "DurationCondition: Scheduler Missing.";
		return;
	}

	deactivate();
	event_ = scheduler->schedule(std::bind(conditionTrigger, 0), duration_);
}

void
DurationCondition::deactivate()
{
	event_.cancel();
}
