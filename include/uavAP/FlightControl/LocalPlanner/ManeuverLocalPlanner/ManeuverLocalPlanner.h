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

#include <cpsCore/cps_object>
#include <cpsCore/Utilities/LockTypes.hpp>

#include <uavAP/Core/DataHandling/Content.hpp>
#include <uavAP/Core/OverrideHandler/OverridableValue.hpp>

#include <uavAP/FlightControl/Controller/ControllerTarget.h>
#include <uavAP/FlightControl/LocalPlanner/ILocalPlanner.h>
#include <uavAP/FlightControl/LocalPlanner/ManeuverLocalPlanner/ManeuverLocalPlannerParams.h>
#include <uavAP/FlightControl/LocalPlanner/ManeuverLocalPlanner/ManeuverLocalPlannerStatus.h>
#include <uavAP/MissionControl/GlobalPlanner/Trajectory.h>

class Packet;

struct SensorData;

class ISensingIO;

class IController;

class IScheduler;

class IPC;

class DataHandling;

class DataPresentation;

class OverrideHandler;

class ManeuverLocalPlanner : public ILocalPlanner,
							 public IRunnableObject,
							 public AggregatableObject<
									 ISensingIO,
									 IController,
									 IScheduler,
									 IPC,
									 DataHandling,
									 DataPresentation,
									 OverrideHandler>,
							 public ConfigurableObject<ManeuverLocalPlannerParams>
{
public:

	static constexpr TypeId typeId = "maneuver";

	ManeuverLocalPlanner() = default;

	void
	setTrajectory(const Trajectory& traj) override;

	Trajectory
	getTrajectory() const override;

	ManeuverLocalPlannerStatus
	getStatus() const;

	bool
	run(RunStage stage) override;

private:

	void
	createLocalPlan(const SensorData& data);

	std::shared_ptr<IPathSection>
	updatePathSection(const SensorData& data);

	void
	nextSection();

	ControllerTarget
	calculateControllerTarget(const SensorData& data,
							  std::shared_ptr<IPathSection> section);

	Optional<Trajectory>
	trajectoryRequest(const DataRequest& request) const;

	void
	onTrajectoryPacket(const Packet& packet);

	void
	onSensorData(const SensorData& sd);

	void
	update();

	mutable Mutex statusMutex_;
	ManeuverLocalPlannerStatus status_;
	ControllerTarget controllerTarget_;

	mutable Mutex trajectoryMutex_;
	Trajectory trajectory_;
	PathSectionIterator currentSection_;

	OverridableValue<FloatingType> velocityOverride_;
	OverridableValue<FloatingType> positionXOverride_;
	OverridableValue<FloatingType> positionYOverride_;
	OverridableValue<FloatingType> positionZOverride_;
	OverridableValue<FloatingType> directionXOverride_;
	OverridableValue<FloatingType> directionYOverride_;
	OverridableValue<FloatingType> curvatureOverride_;
	MaintainableValue<Angle<FloatingType>> headingOverride_{Angle<FloatingType>(0.95), Angle<FloatingType>(0)};
	OverridableValue<FloatingType> climbrateOverride_;
	OverridableValue<FloatingType> controllerTargetVelocityOverride_;
	OverridableValue<FloatingType> controllerTargetClimbAngleOverride_;
	OverridableValue<FloatingType> controllerTargetYawRateOverride_;

};

#endif /* UAVAP_FLIGHTCONTROL_LOCALPLANNER_MANEUVERLOCALPLANNER_MANEUVERLOCALPLANNER_H_ */
