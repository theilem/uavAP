/*
 * ManeuverRatePIDController.h
 *
 *  Created on: Oct 10, 2019
 *      Author: mirco
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_MANEUVERRATEPIDCONTROLLER_MANEUVERRATEPIDCONTROLLER_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_MANEUVERRATEPIDCONTROLLER_MANEUVERRATEPIDCONTROLLER_H_

#include <cpsCore/cps_object>
#include <cpsCore/Configuration/ParameterRef.hpp>
#include <cpsCore/Utilities/LockTypes.hpp>
#include <cpsCore/Utilities/IPC/Subscription.h>

#include <uavAP/Core/SensorData.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>
#include <uavAP/FlightControl/Controller/ControllerTarget.h>
#include <uavAP/FlightControl/Controller/PIDController/IPIDController.h>
#include <uavAP/FlightControl/Controller/PIDController/ManeuverRatePIDController/ManeuverRateCascade.h>
#include <uavAP/FlightControl/Controller/PIDController/ManeuverRatePIDController/ManeuverRatePIDControllerParams.h>

class IScheduler;
class ISensingActuationIO;
class DataHandling;
class IPC;
class DataPresentation;
class Packet;
struct PIDTuning;

class ManeuverRatePIDController: public AggregatableObject<IScheduler, ISensingActuationIO,
		DataHandling, IPC, DataPresentation>,
		public ConfigurableObject<ManeuverRatePIDControllerParams>,
		public IRunnableObject,
		public IPIDController
{
public:

	static constexpr TypeId typeId = "maneuver_rate";

	ManeuverRatePIDController();

	bool
	configure(const Configuration& config);

	bool
	run(RunStage stage) override;

	void
	setControllerTarget(const ControllerTarget& target) override;

	ControllerOutput
	getControllerOutput() override;

	template<typename Config>
	void
	configureParams(Config& c);

	void
	setThrottleLimit(FloatingType maxThrottle);

private:

	void
	onOverridePacket(const Packet& packet);

	void
	tunePID(const PIDTuning& tune);

	Mutex cascadeMutex_;
	ManeuverRateCascade cascade_;

	SensorData sensorData_;
	ControllerTarget target_;
	ControllerOutput output_;

	Subscription overrideSubscription_;

};

template<typename Config>
inline void
ManeuverRatePIDController::configureParams(Config& c)
{
	params.configure(c);

	ParameterRef<ManeuverRateCascade> cascade(cascade_, "cascade", true);

	c & cascade;
}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_MANEUVERRATEPIDCONTROLLER_MANEUVERRATEPIDCONTROLLER_H_ */
