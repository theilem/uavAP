/*
 * ManeuverRateCascade.h
 *
 *  Created on: Oct 10, 2019
 *      Author: mirco
 */

#ifndef UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_MANEUVERRATEPIDCONTROLLER_MANEUVERRATECASCADE_H_
#define UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_MANEUVERRATEPIDCONTROLLER_MANEUVERRATECASCADE_H_

#include <cpsCore/Configuration/ConfigurableObject.hpp>

#include <uavAP/Core/DataHandling/Content.hpp>
#include <uavAP/FlightControl/Controller/ControlElements/ControlEnvironment.h>
#include <uavAP/FlightControl/Controller/ControlElements/Filter/LowPassFilter.h>
#include <uavAP/FlightControl/Controller/PIDController/IPIDCascade.h>
#include <uavAP/FlightControl/Controller/PIDController/ManeuverRatePIDController/ManeuverRateCascadeParams.h>
#include <uavAP/FlightControl/Controller/PIDController/PIDHandling.h>

struct SensorData;
struct ControllerOutput;
struct ControllerTarget;
struct Override;
struct PIDStatus;


class ManeuverRateCascade: public ConfigurableObject<ManeuverRateCascadeParams>, public IPIDCascade
{
public:
	ManeuverRateCascade();

	ManeuverRateCascade(const SensorData& sd, const ControllerTarget& target,
			ControllerOutput& out);

	bool
	configure(const Configuration& config);

	bool
	tunePID(PIDs pid, const Control::PIDParameters& params) override;

	bool
	tuneRollBounds(double min, double max) override;

	bool
	tunePitchBounds(double min, double max) override;

	void
	setThrottleLimit(FloatingType maxThrottle);

	std::map<PIDs, PIDStatus>
	getPIDStatus() override;

	void
	evaluate() override;

	template<typename Config>
	void
	configureParams(Config& c);

	void
	setManeuverOverride(const Override& override);

	Optional<PIDParams>
	getPIDParams(const DataRequest& request);

private:

	FloatingType
	yawrateToRoll(FloatingType yawrate, FloatingType airspeed);

	Control::ControlEnvironment controlEnv_;

	std::map<PIDs, std::shared_ptr<Control::PID>> pids_;

	using AngleConstraint = Control::Constraint<Angle<FloatingType>>;

	std::shared_ptr<AngleConstraint> rollConstraint_;
	std::shared_ptr<AngleConstraint> rollRateTargetConstraint_;
	std::shared_ptr<AngleConstraint> pitchConstraint_;
	std::shared_ptr<Control::Constraint<FloatingType>> throttleConstraint_;

};

template<typename Config>
inline void
ManeuverRateCascade::configureParams(Config& c)
{
	params.configure(c);

	for (auto& pid : pids_)
	{
		ParameterRef<Control::PIDParameters> param(pid.second->getParams(),
				EnumMap<PIDs>::convert(pid.first), true);

		c & param;
	}

	ParameterRef<AngleConstraint> rollConstraint(*rollConstraint_, "roll_constraint", true);
	ParameterRef<AngleConstraint> rollRateConstraint(*rollRateTargetConstraint_,
			"roll_rate_constraint", true);
	ParameterRef<AngleConstraint> pitchConstraint(*pitchConstraint_, "pitch_constraint", true);

	c & rollConstraint;
	c & rollRateConstraint;
	c & pitchConstraint;


}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_MANEUVERRATEPIDCONTROLLER_MANEUVERRATECASCADE_H_ */
