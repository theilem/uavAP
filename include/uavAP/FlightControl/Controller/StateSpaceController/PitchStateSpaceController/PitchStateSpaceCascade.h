//
// Created by seedship on 1/30/21.
//

#ifndef UAVAP_PITCHSTATESPACECASCADE_H
#define UAVAP_PITCHSTATESPACECASCADE_H


#include <cpsCore/Utilities/LockTypes.hpp>
#include <cpsCore/Configuration/ConfigurableObject.hpp>

#include "uavAP/Core/SensorData.h"
#include "uavAP/FlightControl/Controller/ControlElements/ControlEnvironment.h"
#include "uavAP/FlightControl/Controller/PIDController/IPIDCascade.h"


#include <uavAP/Core/DataHandling/Content.hpp>
#include "PitchStateSpaceParams.h"
#include <uavAP/FlightControl/Controller/PIDController/PIDHandling.h>

struct ControllerOutput;
struct ControllerTarget;

class OverrideHandler;

class PitchStateSpaceCascade : public ConfigurableObject<PitchStateSpaceParams>, public IPIDCascade
{
public:
	PitchStateSpaceCascade(const SensorData& sd_enu, const SensorData& sd_ned, const ControllerTarget & target, ControllerOutput& output);

	bool
	tunePID(PIDs pid, const Control::PIDParameters& params) override;

	bool
	tuneRollBounds(FloatingType min, FloatingType max) override;

	bool
	tunePitchBounds(FloatingType min, FloatingType max) override;

	std::map<PIDs, PIDStatus>
	getPIDStatus() override;

	void
	evaluate() override;

	bool
	configure(const Configuration& config);

	void
	setParams(const PitchStateSpaceParams& set);

	void
	setPitchTargetIsClimbAngle(bool isClimbAngle);

	void
	registerOverrides(std::shared_ptr<OverrideHandler> overrideHandler);

	Optional<PIDParams>
	getPIDParams(const DataRequest& request);

	VectorN<6>
	getState() const;

	FloatingType
	getUTrim() const;

	FloatingType
	getThetaTrim() const;

	void
	clearIntegrators();

private:

	void
	configure_(const Configuration& config);

	std::shared_ptr<Control::Constraint<Angle<FloatingType>>> rollConstraint_;
	std::shared_ptr<Control::Constraint<Angle<FloatingType>>> rollRateTargetConstraint_;
	std::shared_ptr<Control::Constraint<Angle<FloatingType>>> pitchConstraint_;

	std::shared_ptr<Control::Integrator> pitchIntegrator_;
	std::shared_ptr<Control::Integrator> velocityIntegrator_;

	// Used to decode if input pitch target is climb angle or pitch
	std::shared_ptr<Control::ManualSwitch> pitchOrClimbAngle_;

	Control::ControlEnvironment controlEnv_;

	std::map<PIDs, std::shared_ptr<Control::PID>> pids_;
	std::map<ControllerOutputs, std::shared_ptr<Control::Output>> ctrlEnvOutputs_;
	FloatingType Ep_;
	FloatingType Ev_;

	const SensorData& sd_ned_;

	// Used for overriding pitch
	Angle<FloatingType> pitchOverrideTarget_;
	std::shared_ptr<Control::ManualSwitch> pitchTarget_;
	// Used for overriding u
	FloatingType velocityOverrideTarget_;
	std::shared_ptr<Control::ManualSwitch> velocityTarget_;

	FloatingType pitchCmd_;
	FloatingType throttleCmd_;
	FloatingType yawCmd_;

//	OverridableValue<FloatingType> pitchOut_;
//	OverridableValue<FloatingType> throttleOut_;
};


#endif //UAVAP_PITCHSTATESPACECASCADE_H
