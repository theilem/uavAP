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
/**
 * @file TestControllerPlugin.cpp
 * @date Sep 20, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief
 */

#include "TestControllerPlugin.h"
#include <uavAP/Core/Scheduler/IScheduler.h>

TestControllerPlugin::TestControllerPlugin() :
		counter_(0)
{
}

void
TestControllerPlugin::notifyAggregationOnUpdate(const Aggregator& agg)
{
	scheduler_.setFromAggregationIfNotSet(agg);
}

void
TestControllerPlugin::setControllerTarget(const ControllerTarget& target)
{
}

bool
TestControllerPlugin::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "Scheduler missing";
			return true;
		}
		break;
	}
	case RunStage::NORMAL:
	{
		auto scheduler = scheduler_.get();

		scheduler->schedule(std::bind(&TestControllerPlugin::testSchedule, this), Milliseconds(0),
				Milliseconds(100));
		break;
	}
	default:
		break;
	}

	return false;
}

void
TestControllerPlugin::testSchedule()
{
	counter_++;
}

int
TestControllerPlugin::evaluateCounter()
{
	return counter_;
}
