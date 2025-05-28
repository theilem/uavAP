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

#include <cpsCore/cps_object>

#include <cpsCore/Utilities/LockTypes.hpp>
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/LinearLocalPlannerStatus.h"

#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/LocalPlanner/ILocalPlanner.h"
#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"

class IScheduler;
class IController;
class ISensingIO;
class Packet;

struct SensorData;
struct ControllerTarget;

struct LinearLocalPlannerParams
{
	Parameter<FloatingType> kAltitude = {1.0, "k_altitude", true};
	Parameter<FloatingType> kHeading = {1.0, "k_heading", true};
	Parameter<FloatingType> kYawrate = {1.0, "k_yawrate", true};
	Parameter<int> period = {0, "period", false};
	Parameter<Angle<FloatingType>> safetyYawRate = {Angle<FloatingType>(10), "safety_yaw_rate", true};

	template <class Configurator>
	void
	configure(Configurator& c)
	{
		c & kAltitude;
		c & kHeading;
		c & kYawrate;
		c & period;
		c & safetyYawRate;
	}
};

class LinearLocalPlanner : public ILocalPlanner, public IRunnableObject, public AggregatableObject<
		ISensingIO, IController, IScheduler>, public ConfigurableObject<
		LinearLocalPlannerParams>
{
public:

	static constexpr TypeId typeId = "linear";

	LinearLocalPlanner();

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
	createLocalPlan(const SensorData& data);

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
