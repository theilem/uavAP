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
 * SimpleMissionPlanner.h
 *
 *  Created on: Jul 27, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_MISSIONPLANNER_SIMPLEMISSIONPLANNER_SIMPLEMISSIONPLANNER_H_
#define UAVAP_MISSIONCONTROL_MISSIONPLANNER_SIMPLEMISSIONPLANNER_SIMPLEMISSIONPLANNER_H_
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/MissionControl/MissionPlanner/Mission.h"
#include "uavAP/MissionControl/MissionPlanner/IMissionPlanner.h"

class IScheduler;
class IGlobalPlanner;

class SimpleMissionPlanner: public IMissionPlanner,
		public IAggregatableObject,
		public IRunnableObject
{
public:

	static constexpr TypeId typeId = "simple";

	SimpleMissionPlanner();

	static std::shared_ptr<SimpleMissionPlanner>
	create(const Configuration& config);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

	void
	missionRequest(const std::string& mission) override;

private:

	void
	publishDefaultMission();

	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<IGlobalPlanner> globalPlanner_;

	Mission defaultMission_;
};

#endif /* UAVAP_MISSIONCONTROL_MISSIONPLANNER_SIMPLEMISSIONPLANNER_SIMPLEMISSIONPLANNER_H_ */
