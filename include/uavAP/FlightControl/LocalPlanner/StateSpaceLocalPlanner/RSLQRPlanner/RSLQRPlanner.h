//
// Created by seedship on 7/25/21.
//

#ifndef UAVAP_RSLQRPLANNER_H
#define UAVAP_RSLQRPLANNER_H

#include <cpsCore/cps_object>
#include <uavAP/FlightControl/LocalPlanner/ILocalPlanner.h>
#include <uavAP/FlightControl/LocalPlanner/LocalPlannerTargets.h>
#include <cpsCore/Utilities/LockTypes.hpp>
#include <uavAP/FlightControl/Controller/ControllerTarget.h>
#include <uavAP/FlightControl/Controller/ControlElements/ControlEnvironment.h>
#include "uavAP/FlightControl/LocalPlanner/StateSpaceLocalPlanner/RSLQRPlanner/RSLQRPlannerParams.h"
#include "uavAP/FlightControl/LocalPlanner/ManeuverLocalPlanner/ManeuverLocalPlannerStatus.h"
#include "uavAP/Core/SensorData.h"

class ISensingIO;
class IScheduler;
class IPC;
class DataHandling;
class DataPresentation;
class OverrideHandler;
class RSLQRController;
enum class PIDs;

class RSLQRPlanner: public ILocalPlanner,
					public IRunnableObject,
					public AggregatableObject<
							ISensingIO,
							IScheduler,
							IPC,
							DataHandling,
							DataPresentation,
							OverrideHandler,
							RSLQRController>,
					public ConfigurableObject<RSLQRPlannerParams>
{
public:
	static constexpr TypeId typeId = "rslqr_planner";

	RSLQRPlanner();

	void
	setTrajectory(const Trajectory& traj) override;

	Trajectory
	getTrajectory() const override;

	bool
	run(RunStage stage) override;


private:

	ManeuverLocalPlannerStatus status_;

	ControllerTarget controllerTarget_;

	mutable Mutex sensorDataMutex_;
	SensorData sensorData_;


	VectorN<2> stateOut_;

	mutable Mutex trajectoryMutex_;
	Trajectory trajectory_;
	PathSectionIterator currentSection_;

	Control::ControlEnvironment controlEnv_;

	mutable Mutex targetMutex_;
	OverridableValue<FloatingType> positionTargetE_;
	OverridableValue<FloatingType> positionTargetN_;
	OverridableValue<FloatingType> positionTargetU_;
	OverridableValue<FloatingType> directionE_;
	OverridableValue<FloatingType> directionN_;
	OverridableValue<Angle<FloatingType>> headingTarget_;
	OverridableValue<FloatingType> controllerTargetVelocity_;
	OverridableValue<FloatingType> controllerTargetPitch_;
	OverridableValue<FloatingType> controllerTargetRoll_;

	FloatingType outputPitch_;
	FloatingType outputRoll_;

	void
	populateControlEnv();

	void
	createLocalPlan(const Vector3& position_enu, FloatingType heading_ned, bool hasGPSFix, FloatingType vz, FloatingType psi_dot);

	ControllerTarget
	calculateControllerTarget(const Vector3& position_enu, double heading_ned,
							  std::shared_ptr<IPathSection> section, FloatingType vz, FloatingType psi_dot);

	std::shared_ptr<IPathSection>
	updatePathSection(const Vector3& position_enu);

	void
	nextSection();

	void onSensorData(const SensorData& sd);

	std::map<PIDs, PIDStatus>
	getStatus() const;
};


#endif //UAVAP_RSLQRPLANNER_H
