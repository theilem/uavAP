//
// Created by seedship on 1/30/21.
//


#include "uavAP/FlightControl/Controller/StateSpaceController/PitchStateSpaceCascade.h"
#include <uavAP/FlightControl/Controller/ControllerTarget.h>
#include <uavAP/Core/OverrideHandler/OverrideHandler.h>

PitchStateSpaceCascade::PitchStateSpaceCascade(const SensorData& sd_enu, const SensorData& sd_ned, const ControllerTarget & target, ControllerOutput& output):
		controlEnv_(&sd_enu.timestamp), sd_ned_(sd_ned), output_(output)
{
	/* Roll Control */
	auto yawrateTarget = controlEnv_.addInput(&target.yawRate);
	auto airspeed = controlEnv_.addInput(&sd_enu.airSpeed);

	auto rollCalc = std::make_shared<Control::CustomFunction2>(yawrateTarget, airspeed, [](FloatingType yawrate, FloatingType airspeed) {
		return -std::atan2(airspeed * yawrate, 9.81);
	});

	rollConstraint_ = std::make_shared<Control::Constraint<Angle<FloatingType>>>(rollCalc);

	auto rollInput = controlEnv_.addInput(&sd_enu.attitude[0]);
	// ENU - QPR
	auto rollRateInput = controlEnv_.addInput(&sd_enu.angularRate[1]);

	Control::PIDParameters defaultParams;
	auto rollPID = controlEnv_.addPID(rollConstraint_, rollInput, rollRateInput, defaultParams);
	rollRateTargetConstraint_ = std::make_shared<Control::Constraint<Angle<FloatingType>>>(rollPID);

	/* Roll Rate Control */
	auto rollRatePID = controlEnv_.addPID(rollRateTargetConstraint_, rollRateInput, defaultParams);

	/* Roll Output */

	auto rollOutConstraint = controlEnv_.addConstraint(rollRatePID, -1, 1);

	auto rollOut = controlEnv_.addOutput(rollOutConstraint, &output.rollOutput);

	pids_.insert(std::make_pair(PIDs::ROLL, rollPID));
	pids_.insert(std::make_pair(PIDs::ROLL_RATE, rollRatePID));

	ctrlEnvOutputs_.insert(std::make_pair(ControllerOutputs::ROLL, rollOut));

	/* State Space Error */
	Control::IntegratorParams defaultIParams;

	/* Pitch */
	auto directTargetVal = controlEnv_.addInput(&target.climbAngle);
	/* Adding Angle of Attack */
	auto aoa = controlEnv_.addInput(&sd_ned.angleOfAttack);
	auto correctedPitch = controlEnv_.addSum(aoa, directTargetVal);

	pitchOrClimbAngle_ = controlEnv_.addManualSwitch(correctedPitch, directTargetVal);
	pitchOrClimbAngle_->switchTo(params.inputClimbAngle.value);


	pitchConstraint_ = std::make_shared<Control::Constraint<Angle<FloatingType>>>(pitchOrClimbAngle_);
	auto overridePitch = controlEnv_.addInput(&pitchOverrideTarget_());
	pitchTarget_ = controlEnv_.addManualSwitch(overridePitch, pitchConstraint_);
	pitchTarget_->switchTo(0);
	auto pitch = controlEnv_.addInput(&sd_enu.attitude[1]);

	// command - target
	auto ep = controlEnv_.addDifference(pitchTarget_, pitch);
	pitchIntegrator_ = controlEnv_.addIntegrator(ep, defaultIParams);
	controlEnv_.addOutput(pitchIntegrator_, &Ep_);

	/* Velocity */
	auto cmdVelocity = controlEnv_.addInput(&target.velocity);
	auto overrideVelocity = controlEnv_.addInput(&velocityOverrideTarget_);
	velocityTarget_ = controlEnv_.addManualSwitch(overrideVelocity, cmdVelocity);
	velocityTarget_->switchTo(0);
	auto velocity = controlEnv_.addInput(&sd_ned_.velocity[0]);
	auto ev = controlEnv_.addDifference(velocityTarget_, velocity);
	velocityIntegrator_ = controlEnv_.addIntegrator(ev, defaultIParams);
	controlEnv_.addOutput(velocityIntegrator_, &Ev_);

	/* Throttle Input & Output */
//	auto throttle = controlEnv_.addInput(&delta_T);
//	auto velocityOffset = controlEnv_.addConstant(1);
//	auto velocityDifference = controlEnv_.addDifference(throttle, velocityOffset);
//	throttleConstraint_ = controlEnv_.addConstraint(velocityDifference, -1, 1);
//	auto throttleOut = controlEnv_.addOutput(throttleConstraint_, &output.throttleOutput);
}

bool
PitchStateSpaceCascade::tunePID(PIDs pid, const Control::PIDParameters& params)
{
	auto it = pids_.find(pid);

	if (it == pids_.end())
	{
		CPSLOG_ERROR << "Unknown pidIndicator. Ignore";
		return false;
	}

	it->second->setParams(params);
	return true;
}

bool
PitchStateSpaceCascade::tuneRollBounds(FloatingType min, FloatingType max)
{
	CPSLOG_ERROR << "Deprecated function called";
	return false;
}

bool
PitchStateSpaceCascade::tunePitchBounds(FloatingType min, FloatingType max)
{
	CPSLOG_ERROR << "Deprecated function called";
	return false;
}

std::map<PIDs, PIDStatus>
PitchStateSpaceCascade::getPIDStatus()
{
	std::map<PIDs, PIDStatus> stati;
	for (const auto& pid : pids_)
	{
		stati.insert(std::make_pair(pid.first, pid.second->getStatus()));
	}
	return stati;
}

void
PitchStateSpaceCascade::evaluate()
{
	assert(sd_ned_.orientation == Orientation::NED);
	assert(sd_ned_.velocity.frame == Frame::BODY);
	controlEnv_.evaluate();

	auto state = getState();

	auto stateOut = -params.k.value * state;

	std::printf("du: %2.8lf actCmd: %2.8lf int: %2.8lf target: %2.8lf current: %2.8lf diff: %2.8lf\n", state[0], stateOut[1], state[5], velocityTarget_->getValue(), sd_ned_.velocity[0], velocityTarget_->getValue() - sd_ned_.velocity[0]);
	std::printf("dθ: %2.8lf actCmd: %2.8lf int: %2.8lf target: %2.8lf current: %2.8lf diff: %2.8lf\n", radToDeg(state[3]), stateOut[0], state[4], radToDeg(pitchTarget_->getValue()), radToDeg(sd_ned_.attitude[1]), radToDeg(pitchTarget_->getValue() - sd_ned_.attitude[1]));

	pitchOut_ = std::clamp(stateOut[0] + params.tE.value, (FloatingType) -1, (FloatingType) 1);
	throttleOut_ = std::clamp(stateOut[1] + params.tT.value, (FloatingType) -1, (FloatingType) 1);

	output_.pitchOutput = pitchOut_;
	output_.throttleOutput = throttleOut_;
}

bool
PitchStateSpaceCascade::configure(const Configuration& config)
{
	auto retVal = ConfigurableObject::configure(config);
	configure_(params.cascade.value);
	if (params.inputClimbAngle())
	{
		CPSLOG_WARN << "PitchStateSpaceCascade cannot yet properly convert climb angle targets to pitch targets. Setting pitch target equal to climb angle target!";
		params.inputClimbAngle.value = false;
	}
	pitchOrClimbAngle_->switchTo(params.inputClimbAngle.value);
	return retVal;
}

void
PitchStateSpaceCascade::setParams(const PitchStateSpaceParams& set)
{
	ConfigurableObject::setParams(set);
	configure_(params.cascade.value);
	if (params.inputClimbAngle())
	{
		CPSLOG_WARN << "PitchStateSpaceCascade cannot yet properly convert climb angle targets to pitch targets. Setting pitch target equal to climb angle target!";
		params.inputClimbAngle.value = false;
	}
	pitchOrClimbAngle_->switchTo(params.inputClimbAngle());
}

void
PitchStateSpaceCascade::setPitchTargetIsClimbAngle(bool isClimbAngle)
{
	params.inputClimbAngle.value = isClimbAngle;
	pitchOrClimbAngle_->switchTo(isClimbAngle);
}

void
PitchStateSpaceCascade::configure_(const Configuration& config)
{
	PropertyMapper<Configuration> c(config);
	for (auto& pid : pids_)
	{
		ParameterRef<Control::PIDParameters> param(pid.second->getParams(),
												   EnumMap<PIDs>::convert(pid.first), true);

		c & param;
	}

	ParameterRef<decltype(rollConstraint_.operator*())> rollConstraint(*rollConstraint_, "roll_constraint", true);
	ParameterRef<decltype(rollRateTargetConstraint_.operator*())> rollRateConstraint(*rollRateTargetConstraint_,
																					 "roll_rate_constraint", true);
	ParameterRef<decltype(pitchConstraint_.operator*())> pitchConstraint(*pitchConstraint_, "pitch_constraint", true);

	ParameterRef<decltype(pitchIntegrator_.operator*())> pitchIntegrator(*pitchIntegrator_, "pitch_integrator", true);
	ParameterRef<decltype(velocityIntegrator_.operator*())> velocityIntegrator(*velocityIntegrator_, "velocity_integrator", true);

	c & rollConstraint;
	c & rollRateConstraint;
	c & pitchConstraint;

	c & pitchIntegrator;
	c & velocityIntegrator;

}

void
PitchStateSpaceCascade::registerOverrides(std::shared_ptr<OverrideHandler> overrideHandler)
{
	overrideHandler->registerOverride("ss/theta", [this](bool enable, FloatingType val){
		pitchOverrideTarget_ = val;
		pitchTarget_->switchTo(enable);
	});
	overrideHandler->registerOverride("ss/u", [this](bool enable, FloatingType val){
		velocityOverrideTarget_ = val;
		velocityTarget_->switchTo(enable);
	});


	for (const auto& it:pids_)
	{
		overrideHandler->registerOverride("pid/" + EnumMap<PIDs>::convert(it.first), [it](bool enable, FloatingType val)
		{ it.second->applyOverride(enable, val); });
		overrideHandler->registerMaintain("pid/" + EnumMap<PIDs>::convert(it.first), [it](bool enable)
		{ it.second->applyMaintain(enable); });
	}
	for (const auto& it:ctrlEnvOutputs_)
	{
		overrideHandler->registerOverride("output/" + EnumMap<ControllerOutputs>::convert(it.first),
										  it.second->getOutputOverridableValue());
		overrideHandler->registerOverride("save_trim/" + EnumMap<ControllerOutputs>::convert(it.first),
										  it.second->getSaveTrimOverridableValue());
		overrideHandler->registerOverride("apply_trim/" + EnumMap<ControllerOutputs>::convert(it.first),
										  it.second->getApplyTrimOverridableValue());
	}
	overrideHandler->registerOverride("output/pitch", pitchOut_);
	overrideHandler->registerOverride("output/throttle", throttleOut_);
}

Optional<PIDParams>
PitchStateSpaceCascade::getPIDParams(const DataRequest& request)
{
	if (request != DataRequest::PID_PARAMS)
		return std::nullopt;

	PIDParams pidParams;

	for (const auto& pid : pids_)
	{
		pidParams.insert(std::make_pair(pid.first, pid.second->getParams()));
	}

	return pidParams;
}

VectorN<6>
PitchStateSpaceCascade::getState() const
{
	//TODO use initializer list when Eigen 3.4 is released
	VectorN<6> state;
	state[0] = sd_ned_.velocity[0] - params.rU.value;
	state[1] = sd_ned_.velocity[1] - params.rW.value;
	state[2] = sd_ned_.angularRate[1];
	state[3] = sd_ned_.attitude[1] - params.rP.value;
	state[4] = Ep_;
	state[5] = Ev_;
	return state;
}

FloatingType
PitchStateSpaceCascade::getUTrim() const
{
	return params.rU.value;
}

FloatingType
PitchStateSpaceCascade::getThetaTrim() const
{
	return params.rP.value;
}
