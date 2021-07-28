//
// Created by seedship on 4/19/21.
//

#ifndef UAVAP_STATESPACEALTITUDEPLANNER_H
#define UAVAP_STATESPACEALTITUDEPLANNER_H


#include <cpsCore/cps_object>
#include <uavAP/FlightControl/LocalPlanner/ILocalPlanner.h>
#include <uavAP/FlightControl/Controller/ControllerTarget.h>
#include <cpsCore/Utilities/LockTypes.hpp>
#include "uavAP/FlightControl/LocalPlanner/StateSpaceLocalPlanner/Longitudinal/StateSpaceCombinedParams.h"
#include <uavAP/Core/OverrideHandler/OverridableValue.hpp>
#include <uavAP/FlightControl/LocalPlanner/ManeuverLocalPlanner/ManeuverLocalPlannerStatus.h>

class ISensingIO;
class IScheduler;
class IPC;
class DataHandling;
class DataPresentation;
class OverrideHandler;
class Packet;
class PitchStateSpaceController;
struct SensorData;

class StateSpaceAltitudePlanner: public ILocalPlanner,
								 public IRunnableObject,
								 public AggregatableObject<
										 ISensingIO,
										 IScheduler,
										 IPC,
										 DataHandling,
										 DataPresentation,
										 OverrideHandler,
										 PitchStateSpaceController>,
								 public ConfigurableObject<StateSpaceCombinedParams>
{

public:
	static constexpr TypeId typeId = "statespace_altitude";

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
	createLocalPlan(const SensorData& sd);

	std::shared_ptr<IPathSection>
	updatePathSection(const Vector3& position);

	void
	nextSection();

	ControllerTarget
	calculateControllerTarget(const Vector3& position, double heading,
							  std::shared_ptr<IPathSection> section);

	void
	onTrajectoryPacket(const Packet& packet);

	void
	update();

	mutable Mutex statusMutex_;
	ManeuverLocalPlannerStatus status_;
	ControllerTarget controllerTarget_;

	mutable Mutex trajectoryMutex_;
	Trajectory trajectory_;
	PathSectionIterator currentSection_;

	OverridableValue<FloatingType> velocity_;
	OverridableValue<FloatingType> positionX_;
	OverridableValue<FloatingType> positionY_;
	OverridableValue<FloatingType> positionZ_;
	OverridableValue<FloatingType> directionX_;
	OverridableValue<FloatingType> directionY_;
	OverridableValue<FloatingType> curvature_;
	OverridableValue<Angle<FloatingType>> heading_;
	OverridableValue<FloatingType> pitch_;
	OverridableValue<FloatingType> controllerTargetVelocity_;
	OverridableValue<FloatingType> controllerTargetPitch_;
	OverridableValue<FloatingType> controllerTargetYawRate_;

	FloatingType controllerThetaRef_;
};


#endif //UAVAP_STATESPACEALTITUDEPLANNER_H
