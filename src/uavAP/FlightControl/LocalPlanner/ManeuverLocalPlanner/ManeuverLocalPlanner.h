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

#ifndef UAVAP_FLIGHTCONTROL_LOCALPLANNER_MANEUVERLOCALPLANNER_MANEUVERLOCALPLANNER_H_
#define UAVAP_FLIGHTCONTROL_LOCALPLANNER_MANEUVERLOCALPLANNER_MANEUVERLOCALPLANNER_H_

#include <boost/property_tree/ptree.hpp>

#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/protobuf/messages/LocalPlanner.pb.h>
#include <uavAP/Core/Runner/IRunnableObject.h>
#include <uavAP/FlightControl/Controller/ControllerTarget.h>
#include <uavAP/FlightControl/LocalPlanner/ILocalPlanner.h>
#include <uavAP/MissionControl/GlobalPlanner/Trajectory.h>
#include <uavAP/MissionControl/ManeuverPlanner/Override.h>

class Packet;
struct SensorData;

class ISensingActuationIO;
class IController;
class IScheduler;
class IPC;

class ManeuverLocalPlanner: public ILocalPlanner, public IRunnableObject, public IAggregatableObject
{
public:

	static constexpr TypeId typeId = "maneuver";

	ManeuverLocalPlanner();

	bool
	configure(const boost::property_tree::ptree& config);

	ADD_CREATE_WITH_CONFIG(ManeuverLocalPlanner)

	void
	setTrajectory(const Trajectory& traj) override;

	Trajectory
	getTrajectory() const override;

	LocalPlannerStatus
	getStatus() const;

	bool
	tune(const LocalPlannerParams& params);

	bool
	run(RunStage stage) override;

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

private:

	void
	createLocalPlan(const Vector3& position, double heading, bool hasGPSFix, uint32_t seqNum);

	std::shared_ptr<IPathSection>
	updatePathSection(const Vector3& position);

	void
	nextSection();

	ControllerTarget
	calculateControllerTarget(const Vector3& position, double heading, std::shared_ptr<IPathSection> section);

	void
	onTrajectoryPacket(const Packet& packet);

	void
	onSensorData(const SensorData& sd);

	void
	onOverridePacket(const Packet& packet);

	void
	update();

	ObjectHandle<ISensingActuationIO> sensing_;
	ObjectHandle<IController> controller_;
	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<IPC> ipc_;

	std::mutex paramsMutex_;
	ManeuverLocalPlannerParams params_;
	unsigned int period_;
	ControllerTarget safetyTarget_;

	mutable std::mutex statusMutex_;
	ManeuverLocalPlannerStatus status_;
	ControllerTarget controllerTarget_;

	mutable std::mutex trajectoryMutex_;
	Trajectory trajectory_;
	PathSectionIterator currentSection_;

	//Overrides
	Override::LocalPlannerOverrides plannerOverrides_;
	Override::ControllerTargetOverrides targetOverrides_;
	std::mutex overrideMutex_;

};

#endif /* UAVAP_FLIGHTCONTROL_LOCALPLANNER_MANEUVERLOCALPLANNER_MANEUVERLOCALPLANNER_H_ */
