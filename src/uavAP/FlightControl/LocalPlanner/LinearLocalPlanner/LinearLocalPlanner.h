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
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/ILinearPlannerImpl.h"
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/LinearLocalPlannerStatus.h"

#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/LocalPlanner/ILocalPlanner.h"
#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/LockTypes.h"

class FlightControlData;
class IScheduler;
class IController;
class IPC;
class ISensingActuationIO;
class Packet;
struct SensorData;
struct ControllerTarget;

class LinearLocalPlanner: public ILocalPlanner, public IRunnableObject, public AggregatableObject<ISensingActuationIO,IController, IScheduler, IPC>
{
public:

	static constexpr TypeId typeId = "linear";

	LinearLocalPlanner();

	bool
	configure(const Configuration& config);

	ADD_CREATE_WITH_CONFIG(LinearLocalPlanner)

	bool
	run(RunStage stage) override;

	ControllerTarget
	getControllerTarget();

	void
	setTrajectory(const Trajectory& traj) override;

	Trajectory
	getTrajectory() const override;

	std::shared_ptr<ILinearPlannerImpl>
	getImpl();

	LocalPlannerStatus
	getStatus() const override;

	bool
	tune(const LocalPlannerParams& params) override;

private:

	void
	createLocalPlan(const Vector3& position, double heading, bool hasGPSFix, uint32_t seqNum);

	void
	nextSection();

	void
	onTrajectoryPacket(const Packet& packet);

	void
	onSensorData(const SensorData& sd);

	void
	update();

	std::shared_ptr<ILinearPlannerImpl> localPlannerImpl_;

	Trajectory trajectory_;
	PathSectionIterator currentSection_;
	bool inApproach_;
	std::mutex trajectoryMutex_;
	ControllerTarget controllerTarget_;

	bool airplane_;
	unsigned int period_;
	uint8_t currentPathSectionIdx_;
};

#endif /* UAVAP_FLIGHTCONTROL_LOCALPLANNER_LINEARLOCALPLANNER_LINEARLOCALPLANNER_H_ */
