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
 *  @file         LinearLocalPlanner.h
 *  @author  Mirco Theile
 *  @date      27 June 2017
 *  @brief      UAV Autopilot Linear Local Planner Header File
 *
 *  Description
 */

#ifndef UAVAP_FLIGHTCONTROL_LOCALPLANNER_LINEARLOCALPLANNER_LINEARLOCALPLANNER_H_
#define UAVAP_FLIGHTCONTROL_LOCALPLANNER_LINEARLOCALPLANNER_LINEARLOCALPLANNER_H_

#include <boost/property_tree/ptree.hpp>
#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/ILinearPlannerImpl.h"
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/LinearLocalPlannerStatus.h"

#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/LocalPlanner/ILocalPlanner.h"
#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/LockTypes.h"
#include "uavAP/Core/DataPresentation/Content.h"

class FlightControlData;
class IScheduler;
class IController;
enum class Content;
enum class Target;

template<typename C, typename T>
class IDataPresentation;
class IPC;
class SensingActuationIO;
struct ControllerTarget;

class LinearLocalPlanner: public ILocalPlanner, public IRunnableObject, public IAggregatableObject
{
public:

	LinearLocalPlanner();

	static std::shared_ptr<LinearLocalPlanner>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	bool
	run(RunStage stage) override;

	ControllerTarget
	getControllerTarget();

	void
	setTrajectory(const Trajectory& traj) override;

	Trajectory
	getTrajectory() const override;

	void
	notifyAggregationOnUpdate(Aggregator& agg) override;

	std::shared_ptr<ILinearPlannerImpl>
	getImpl();

	LocalPlannerStatus
	getStatus();

private:

	void
	createLocalPlan(const SensorData& data);

	void
	nextSection();

	void
	onTrajectoryPacket(const Packet& packet);

	ObjectHandle<SensingActuationIO> sensing_;
	ObjectHandle<IController> controller_;
	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<IPC> ipc_;
	ObjectHandle<IDataPresentation<Content,Target>> dataPresentation_;

	std::shared_ptr<ILinearPlannerImpl> localPlannerImpl_;

	Trajectory trajectory_;
	PathSectionIterator currentSection_;
	std::mutex trajectoryMutex_;
	ControllerTarget controllerTarget_;

	bool airplane_;
	uint8_t currentPathSectionIdx_;
};

#endif /* UAVAP_FLIGHTCONTROL_LOCALPLANNER_LINEARLOCALPLANNER_LINEARLOCALPLANNER_H_ */
