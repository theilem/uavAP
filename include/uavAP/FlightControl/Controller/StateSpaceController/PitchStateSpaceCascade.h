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
#include "uavAP/FlightControl/Controller/StateSpaceController/PitchStateSpaceParams.h"
#include <uavAP/FlightControl/Controller/PIDController/PIDHandling.h>
#include <uavAP/MissionControl/ManeuverPlanner/Override.h>

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
	registerOverrides(std::shared_ptr<OverrideHandler> overrideHandler);

	Optional<PIDParams>
	getPIDParams(const DataRequest& request);

private:

	void
	configure_(const Configuration& config);

	std::shared_ptr<Control::Constraint<Angle<FloatingType>>> rollConstraint_;
	std::shared_ptr<Control::Constraint<Angle<FloatingType>>> rollRateTargetConstraint_;
	std::shared_ptr<Control::Constraint<Angle<FloatingType>>> pitchConstraint_;

	std::shared_ptr<Control::Integrator> pitchIntegrator_;
	std::shared_ptr<Control::Integrator> velocityIntegrator_;

	Control::ControlEnvironment controlEnv_;
	Eigen::Matrix<FloatingType, 2, 6> k_;

	std::map<PIDs, std::shared_ptr<Control::PID>> pids_;
	std::map<ControllerOutputs, std::shared_ptr<Control::Output>> ctrlEnvOutputs_;
	FloatingType Ep_;
	FloatingType Ev_;

	const SensorData& sd_ned_;
	ControllerOutput& output_;
	const ControllerTarget& target_;


	Angle<FloatingType> pitchOverrideTarget_;
	FloatingType velocityOverrideTarget_;

	std::shared_ptr<Control::ManualSwitch> pitchTarget_;
	std::shared_ptr<Control::ManualSwitch> velocityTarget_;

	OverridableValue<FloatingType> pitchOut_;
	OverridableValue<FloatingType> throttleOut_;
};


#endif //UAVAP_PITCHSTATESPACECASCADE_H
