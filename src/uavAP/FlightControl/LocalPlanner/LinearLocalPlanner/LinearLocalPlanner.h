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

#include <uavAP/Core/Object/AggregatableObject.hpp>
#include <uavAP/Core/PropertyMapper/ConfigurableObject.hpp>
#include <uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/LinearLocalPlannerParams.h>
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/LinearLocalPlannerStatus.h"

#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/LocalPlanner/ILocalPlanner.h"
#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/LockTypes.h"

class IScheduler;
class IController;
class ISensingActuationIO;
class Packet;
struct SensorData;
struct ControllerTarget;

class LinearLocalPlanner: public ILocalPlanner, public IRunnableObject, public AggregatableObject<
		ISensingActuationIO, IController, IScheduler>, public ConfigurableObject<
		LinearLocalPlannerParams>
{
public:

	static constexpr TypeId typeId = "linear";

	LinearLocalPlanner();

	static std::shared_ptr<LinearLocalPlanner>
	create(const Configuration& config);

	bool
	run(RunStage stage) override;

	ControllerTarget
	getControllerTarget();

	void
	setTrajectory(const Trajectory& traj) override;

	Trajectory
	getTrajectory() const override;

private:

	void
	createLocalPlan(const Vector3& position, FloatingType heading, bool hasGPSFix, uint32_t seqNum);

	void
	nextSection();

	void
	onSensorData(const SensorData& sd);

	void
	update();

	ControllerTarget
	evaluate(const Vector3& position, FloatingType heading, std::shared_ptr<IPathSection> section);

	FloatingType headingTarget_;

	Trajectory trajectory_;
	PathSectionIterator currentSection_;
	bool inApproach_;
	Mutex trajectoryMutex_;
	ControllerTarget controllerTarget_;

	uint8_t currentPathSectionIdx_;
};

#endif /* UAVAP_FLIGHTCONTROL_LOCALPLANNER_LINEARLOCALPLANNER_LINEARLOCALPLANNER_H_ */
