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
#include <uavAP/FlightControl/Controller/PIDController/PIDHandling.h>

struct SensorData;
struct ControllerOutput;
struct ControllerTarget;
struct PIDStatus;

class OverrideHandler;

class ManeuverRateCascade: public ConfigurableObject<PlaceholderParams>, public IPIDCascade
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

	void
	setThrottleOverride(FloatingType throttleOverride);

	void
	disableThrottleOverride();

	std::map<PIDs, PIDStatus>
	getPIDStatus() override;

	void
	evaluate() override;

	template<typename Config>
	void
	configureParams(Config& c);

	Optional<PIDParams>
	getPIDParams(const DataRequest& request);

	void
	registerOverrides(std::shared_ptr<OverrideHandler> overrideHandler);

private:

	FloatingType
	yawrateToRoll(FloatingType yawrate, FloatingType airspeed);

	Control::ControlEnvironment controlEnv_;

	std::map<PIDs, std::shared_ptr<Control::PID>> pids_;
	std::map<ControllerOutputs, std::shared_ptr<Control::Output>> outputs_;

	using AngleConstraint = Control::Constraint<Angle<FloatingType>>;

	std::shared_ptr<AngleConstraint> rollConstraint_;
	std::shared_ptr<AngleConstraint> rollRateTargetConstraint_;
	std::shared_ptr<AngleConstraint> pitchConstraint_;
	std::shared_ptr<Control::Constraint<FloatingType>> throttleConstraint_;

	std::shared_ptr<Control::ManualSwitch> throttleSwitch_;

	std::optional<FloatingType> throttleOverride_;
	FloatingType throttleOverrideValue_;
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

	for (auto& output : outputs_)
	{
		ParameterRef<FloatingType> param(output.second->getTrimAlpha(),
										 EnumMap<ControllerOutputs>::convert(output.first) + "_alpha", true);

		c & param;
	}

	ParameterRef<AngleConstraint> rollConstraint(*rollConstraint_, "roll_constraint", true);
	ParameterRef<AngleConstraint> rollRateConstraint(*rollRateTargetConstraint_,
													 "roll_rate_constraint", true);
	ParameterRef<AngleConstraint> pitchConstraint(*pitchConstraint_, "pitch_constraint", true);
	ParameterRef<std::optional<FloatingType>> throttleOverride(throttleOverride_, "throttle_override", false);

	c & rollConstraint;
	c & rollRateConstraint;
	c & pitchConstraint;
	c & throttleOverride;

}

#endif /* UAVAP_FLIGHTCONTROL_CONTROLLER_PIDCONTROLLER_MANEUVERRATEPIDCONTROLLER_MANEUVERRATECASCADE_H_ */
