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

#include <cpsCore/Utilities/Scheduler/IScheduler.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include "uavAP/MissionControl/ConditionManager/ConditionManager.h"
#include "uavAP/MissionControl/ConditionManager/Condition/SteadyStateCondition.h"

SteadyStateCondition::SteadyStateCondition() :
		connection_(), steadyState_(true), trigger_(), event_(), minimumDuration_(
				Milliseconds(500)), afterMinimumDuration_(false)
{
}

std::shared_ptr<SteadyStateCondition>
SteadyStateCondition::create(const Configuration& config)
{
	auto steadyStateCondition = std::make_shared<SteadyStateCondition>();

	if (!steadyStateCondition->configure(config))
	{
		CPSLOG_ERROR << "SteadyStateCondition: Failed to Load Config.";
	}

	return steadyStateCondition;
}

bool
SteadyStateCondition::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);

	pm.add<bool>("steady_state", steadyState_, true);
	pm.add("minimum_duration", minimumDuration_, false);

	dp_ = nullptr;

	return pm.map();
}

void
SteadyStateCondition::activate(ConditionManager* conditionManager,
		const ConditionTrigger& conditionTrigger)
{
	deactivate();
	auto scheduler = conditionManager->getScheduler().lock();
	dp_ = conditionManager->getDataPresentation().lock();

	if (!scheduler)
	{
		CPSLOG_ERROR << "SteadyStateCondition: Scheduler Missing.";
		return;
	}

	if (!dp_)
	{
		CPSLOG_ERROR << "SteadyStateCondition: Data Presentation Missing.";
		return;
	}

	event_ = scheduler->schedule(std::bind(&SteadyStateCondition::minDuration, this),
			minimumDuration_);

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
	if (!afterMinimumDuration_ || !dp_)
	{
		return;
	}

	bool inSteadyState = dp_->deserialize<bool>(packet);

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
