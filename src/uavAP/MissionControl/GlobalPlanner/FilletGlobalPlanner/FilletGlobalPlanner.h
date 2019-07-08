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
 * SimpleFlightPlanner.h
 *
 *  Created on: Jun 7, 2017
 *      Author: mircot
 */

#ifndef FLIGHTPLANNER_SIMPLEFLIGHTPLANNER_SIMPLEFLIGHTPLANNER_H_
#define FLIGHTPLANNER_SIMPLEFLIGHTPLANNER_SIMPLEFLIGHTPLANNER_H_

#include "uavAP/MissionControl/GlobalPlanner/IGlobalPlanner.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"

#include <boost/property_tree/ptree.hpp>
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"


class FilletGlobalPlanner: public IGlobalPlanner, public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "fillet";

	FilletGlobalPlanner();

	static std::shared_ptr<FilletGlobalPlanner>
	create(const Configuration& config);

	bool
	configure(const Configuration& config);

	void
	collectWaypoint(const VectorWrapper<Waypoint>& wp);

	void
	setMission(const Mission& mission) override;

	Mission
	getMission() const override;

	bool
	run(RunStage stage) override;

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

private:

	double filletRadius_;

	Mission mission_;

	ObjectHandle<IPC> ipc_;

	Publisher trajectoryPublisher_;

};

#endif /* FLIGHTPLANNER_SIMPLEFLIGHTPLANNER_SIMPLEFLIGHTPLANNER_H_ */
