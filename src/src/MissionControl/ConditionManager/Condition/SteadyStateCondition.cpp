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
 * SteadyStateCondition.cpp
 *
 *  Created on: Jul 29, 2018
 *      Author: simonyu
 */

#include <uavAP/Core/Scheduler/IScheduler.h>
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/MissionControl/ConditionManager/ConditionManager.h"
#include "uavAP/MissionControl/ConditionManager/Condition/SteadyStateCondition.h"
#include "uavAP/Core/DataPresentation/BinarySerialization.hpp"

SteadyStateCondition::SteadyStateCondition() :
		connection_(), steadyState_(true), event_(), minimumDuration_(Milliseconds(500)), trigger_(), afterMinimumDuration_(false)
{
}

std::shared_ptr<SteadyStateCondition>
SteadyStateCondition::create(const boost::property_tree::ptree& config)
{
	auto steadyStateCondition = std::make_shared<SteadyStateCondition>();

	if (!steadyStateCondition->configure(config))
	{
		APLOG_ERROR << "SteadyStateCondition: Failed to Load Config.";
	}

	return steadyStateCondition;
}

bool
SteadyStateCondition::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);

	pm.add<bool>("steady_state", steadyState_, true);
	pm.add("minimum_duration", minimumDuration_, false);

	return pm.map();
}

void
SteadyStateCondition::activate(ConditionManager* conditionManager, const ConditionTrigger& conditionTrigger)
{
	deactivate();
	auto scheduler = conditionManager->getScheduler().lock();

	if (!scheduler)
	{
		APLOG_ERROR << "SteadyStateCondition: Scheduler Missing.";
		return;
	}

	event_ = scheduler->schedule(std::bind(&SteadyStateCondition::minDuration, this), minimumDuration_);

	connection_ = conditionManager->subscribeOnSteadyState(
		std::bind(&SteadyStateCondition::onSteadyState, this, std::placeholders::_1));
	trigger_ = conditionTrigger;
}

void
SteadyStateCondition::deactivate()
{
	event_.cancel();
	connection_.disconnect();
	afterMinimumDuration_ = false;
}

void
SteadyStateCondition::onSteadyState(const Packet& packet)
{
	if (!afterMinimumDuration_)
		return;

	bool inSteadyState = dp::deserialize<bool>(packet);

	if (inSteadyState == steadyState_)
	{
		deactivate();
		trigger_(0);
	}
}

void
SteadyStateCondition::minDuration()
{
	afterMinimumDuration_ = true;
}
