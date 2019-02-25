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
 * SplineGlobalPlanner.h
 *
 *  Created on: Dec 16, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_GLOBALPLANNER_SPLINEGLOBALPLANNER_SPLINEGLOBALPLANNER_H_
#define UAVAP_MISSIONCONTROL_GLOBALPLANNER_SPLINEGLOBALPLANNER_SPLINEGLOBALPLANNER_H_
#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/MissionControl/GlobalPlanner/IGlobalPlanner.h"
#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"

class IPC;
class ILocalPlanner;

class SplineGlobalPlanner: public IGlobalPlanner, public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "spline";

	SplineGlobalPlanner();

	static std::shared_ptr<IGlobalPlanner>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	bool
	run(RunStage stage) override;

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	void
	setMission(const Mission& mission) override;

	Mission
	getMission() const override;

private:

	Trajectory
	createNaturalSplines(const Mission& mission);

	Trajectory
	createCatmulRomSplines(const Mission& mission);

	double orbitRadius_;
	double tau_;
	uint8_t inclusionLength_;
	bool smoothenZ_;
	bool naturalSplines_;

	Mission mission_;

	ObjectHandle<IPC> ipc_;
	ObjectHandle<ILocalPlanner> localPlanner_;

	Publisher trajectoryPublisher_;

};

#endif /* UAVAP_MISSIONCONTROL_GLOBALPLANNER_SPLINEGLOBALPLANNER_SPLINEGLOBALPLANNER_H_ */
