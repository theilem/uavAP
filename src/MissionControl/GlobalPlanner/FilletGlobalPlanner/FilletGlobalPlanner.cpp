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
 * SimpleFlightPlanner.cpp
 *
 *  Created on: Jun 7, 2017
 *      Author: mircot
 */

#include "uavAP/MissionControl/GlobalPlanner/FilletGlobalPlanner/FilletGlobalPlanner.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Line.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Curve.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Orbit.h"
#include "cpsCore/Utilities/IPC/IPC.h"
#include "cpsCore/Utilities/DataPresentation/DataPresentation.h"

FilletGlobalPlanner::FilletGlobalPlanner() :
		filletRadius_(0)
{

}

std::shared_ptr<FilletGlobalPlanner>
FilletGlobalPlanner::create(const Configuration& config)
{
	auto planner = std::make_shared<FilletGlobalPlanner>();
	if (!planner->configure(config))
	{
		CPSLOG_ERROR << "FilletGlobalPlanner Configuration failed.";
	}
	return planner;
}

bool
FilletGlobalPlanner::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);
	pm.add<FloatingType>("fillet_radius", filletRadius_, true);

	return pm.map();
}

bool
FilletGlobalPlanner::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!checkIsSet<IPC, DataPresentation>())
		{
			CPSLOG_ERROR << "FilletGlobalPlanner: Missing dependencies.";
			return true;
		}
		auto ipc = get<IPC>();
		trajectoryPublisher_ = ipc->publishPackets("trajectory");
		break;
	}
	case RunStage::NORMAL:
	{
		break;
	}
	case RunStage::FINAL:
		break;
	default:
		break;
	}
	return false;
}

void
FilletGlobalPlanner::setMission(const Mission& mission)
{
	mission_ = mission;
	bool infinite = true;
	PathSections traj;
	if (mission.waypoints().size() == 1)
	{
		infinite = false;
	}
	//Avoid reallocation and iterator missmatch during iterations by reserving
	traj.reserve(mission.waypoints().size() * 2);
	//Connect waypoints with lines first
	for (auto it = mission.waypoints().begin(); it != mission.waypoints().end(); ++it)
	{
		auto nextIt = it + 1;
		if (nextIt == mission.waypoints().end())
		{
			if (!infinite)
				break;

			nextIt = mission.waypoints().begin();
		}
		EigenLine line = EigenLine::Through(it->location(), nextIt->location());
		double vel = mission_.velocity();
		if (nextIt->velocity())
			vel = *nextIt->velocity();
		traj.push_back(std::make_shared<Line>(line, nextIt->location(), vel));
	}

	//Now cut the lines with fillets
	for (auto it = traj.begin(); it != traj.end(); ++it)
	{
		auto nextIt = it + 1;
		if (nextIt == traj.end())
		{
			if (!infinite)
				break;

			nextIt = traj.begin();
		}
		auto line = dynamic_cast<Line*>(it->get());
		auto nextLine = dynamic_cast<Line*>((nextIt)->get());
		//Directions are normalized, negative due to opposite directions
		double dotProduct = -nextLine->direction().dot(line->getDirection());
		//Should be between -1 and 1 but float rounding errors might overshoot
		double alpha = std::acos(dotProduct);
		//Distance to waypoint: d = r / sin(alpha/2)
		if (fabs(alpha) > (9. / 10.) * M_PI)
			continue; //We don't need a curve as the angle is smooth enough
		double d = filletRadius_ / std::sin(alpha / 2);
		//Direction from waypoint to center is the bisecting of both lines
		Vector3 dir = (nextLine->getDirection() - line->getDirection()) / 2.0;

		Vector3 center = nextLine->origin() + d * dir;
		Vector3 normal = (*nextIt)->getDirection().cross(-(*it)->getDirection());

		//Set end point of line as tangent point to curve
		line->setEndPoint(line->projection(center));
		//Set end point of curve to tangent point of next line to curve
		auto endCurve = nextLine->projection(center);

		auto curve = std::make_shared<Curve>(center, normal, endCurve, nextLine->getVelocity());

		traj.insert(nextIt, curve);
		//Step over inserted curve
		++it;
	}

	if (!infinite)
	{
		Vector3 center;
		double vel;
		//Make orbit in the end to not just crash down
		if (traj.empty())
		{
			center = mission.waypoints().back().location();
			vel = mission.velocity();
		}
		else
		{
			center = traj.back()->getEndPoint();
			vel = traj.back()->getVelocity();
		}
		//Make orbit parallel to ground at the endPoint of  the last line
		auto orbit = std::make_shared<Orbit>(center, Vector3(0, 0, 1), filletRadius_, vel);
		traj.push_back(orbit);
	}

	if (auto dp = get<DataPresentation>())
	{
		CPSLOG_DEBUG << "Send trajectory";
		auto packet = dp->serialize(Trajectory(traj, mission.infinite()));
		trajectoryPublisher_.publish(packet);
	}
}

Mission
FilletGlobalPlanner::getMission() const
{
	return mission_;
}
