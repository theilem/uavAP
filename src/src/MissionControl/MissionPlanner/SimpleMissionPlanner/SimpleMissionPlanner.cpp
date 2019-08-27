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
 * SimpleMissionPlanner.cpp
 *
 *  Created on: Jul 27, 2017
 *      Author: mircot
 */
#include "uavAP/MissionControl/MissionPlanner/SimpleMissionPlanner/SimpleMissionPlanner.h"
#include "uavAP/MissionControl/GlobalPlanner/IGlobalPlanner.h"
#include "uavAP/Core/Scheduler/IScheduler.h"

SimpleMissionPlanner::SimpleMissionPlanner()
{
	defaultMission_.waypoints.push_back(Waypoint(Vector3(4435643.06, 367866.57, -280)));
	defaultMission_.waypoints.push_back(Waypoint(Vector3(4435329.03, 367569.06, -280)));
	defaultMission_.waypoints.push_back(Waypoint(Vector3(4435242.13, 367700.47, -280)));
	defaultMission_.waypoints.push_back(Waypoint(Vector3(4435559.95, 368010.59, -280)));
	defaultMission_.infinite = true;
	defaultMission_.velocity = 15;
}

void
SimpleMissionPlanner::notifyAggregationOnUpdate(const Aggregator& agg)
{
	scheduler_.setFromAggregationIfNotSet(agg);
	globalPlanner_.setFromAggregationIfNotSet(agg);
}

bool
SimpleMissionPlanner::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "SimpleMissionPlanner: Scheduler missing.";
			return true;
		}
		if (!globalPlanner_.isSet())
		{
			APLOG_ERROR << "SimpleMissionPlanner: Global Planner missing.";
			return true;
		}
		break;
	}
	case RunStage::NORMAL:
	{
		auto scheduler = scheduler_.get();
		scheduler->schedule(std::bind(&SimpleMissionPlanner::publishDefaultMission, this),
				Milliseconds(0));
		break;
	}
	case RunStage::FINAL:
		break;
	default:
		break;
	}
	return false;
}

std::shared_ptr<SimpleMissionPlanner>
SimpleMissionPlanner::create(const Configuration& config)
{
	return std::make_shared<SimpleMissionPlanner>();
}

void
SimpleMissionPlanner::publishDefaultMission()
{
	auto gp = globalPlanner_.get();
	if (!gp)
	{
		APLOG_ERROR << "Global Planner missing.";
		return;
	}

	gp->setMission(defaultMission_);
}

void
SimpleMissionPlanner::missionRequest(const std::string& mission)
{
	APLOG_ERROR << "Mission selection not possible in simple mission planner";
}
