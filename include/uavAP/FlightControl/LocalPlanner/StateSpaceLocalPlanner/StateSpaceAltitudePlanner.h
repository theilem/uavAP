//
// Created by seedship on 4/19/21.
//

#ifndef UAVAP_STATESPACEALTITUDEPLANNER_H
#define UAVAP_STATESPACEALTITUDEPLANNER_H


#include <cpsCore/cps_object>
#include <uavAP/FlightControl/LocalPlanner/ILocalPlanner.h>
#include <uavAP/FlightControl/Controller/ControllerTarget.h>
#include <cpsCore/Utilities/LockTypes.hpp>
#include "uavAP/FlightControl/LocalPlanner/StateSpaceLocalPlanner/StateSpaceAltitudePlannerParams.h"
#include "StateSpaceAltitudePlannerStatus.h"
#include <uavAP/Core/OverrideHandler/OverridableValue.hpp>
#include <uavAP/FlightControl/Controller/StateSpaceController/PitchStateSpaceController.h>

class ISensingIO;
class IController;
class IScheduler;
class IPC;
class DataHandling;
class DataPresentation;
class OverrideHandler;
class Packet;
class SensorData;

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
								 public ConfigurableObject<StateSpaceAltitudePlannerParams>
{

public:
	static constexpr TypeId typeId = "statespace_altitude";

	StateSpaceAltitudePlanner();

	void
	setTrajectory(const Trajectory& traj) override;

	Trajectory
	getTrajectory() const override;

	StateSpaceAltitudePlannerStatus
	getStatus() const;

	bool
	run(RunStage stage) override;

	bool
	configure(const Configuration &c);

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
	StateSpaceAltitudePlannerStatus status_;
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

	FloatingType controllerURef_;
	FloatingType controllerThetaRef_;
};


#endif //UAVAP_STATESPACEALTITUDEPLANNER_H
