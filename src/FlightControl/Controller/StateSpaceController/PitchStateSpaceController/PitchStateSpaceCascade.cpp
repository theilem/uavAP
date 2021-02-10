//
// Created by seedship on 1/30/21.
//


#include "uavAP/FlightControl/Controller/StateSpaceController/PitchStateSpaceCascade.h"
#include <uavAP/FlightControl/Controller/ControllerTarget.h>

PitchStateSpaceCascade::PitchStateSpaceCascade(const SensorData& sd_enu, const SensorData& sd_ned, const ControllerTarget & target, ControllerOutput& output):
		controlEnv_(&sd_enu.timestamp), sd_ned_(sd_ned), output_(output), target_(target)//, a_(a.data()), b_(b.data()), k_(k.data())
{
	/* Roll Control */
	auto yawrateTarget = controlEnv_.addInput(&target.yawRate);
	auto airspeed = controlEnv_.addInput(&sd_enu.airSpeed);

	auto rollCalc = std::make_shared<Control::CustomFunction2>(yawrateTarget, airspeed, [](FloatingType yawrate, FloatingType airspeed) {
//		CPSLOG_WARN << airspeed << " " << yawrate;
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

	/* State Space Error */
	/* Pitch */
	auto cmdPitch = controlEnv_.addInput(&target.climbAngle);
	auto pitch = controlEnv_.addInput(&sd_enu.attitude[1]);
	// command - target
	auto ep = controlEnv_.addDifference(cmdPitch, pitch);
	auto cEp = controlEnv_.addIntegrator(ep, 0, -5, 5);
	controlEnv_.addOutput(cEp, &Ep_);

	/* Velocity */
	auto cmdVelocity = controlEnv_.addInput(&target.velocity);
	auto velocity = controlEnv_.addInput(&sd_ned_.velocity[0]);
	auto ev = controlEnv_.addDifference(cmdVelocity, velocity);
	auto cEv = controlEnv_.addIntegrator(ev, 0, -1, 1);
	controlEnv_.addOutput(cEv, &Ev_);

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

	// Kind of optional when roll is 0
//	assert(sd_ned_.angularRate.frame == Frame::BODY);
	controlEnv_.evaluate();

	//TODO use initializer list when Eigen 3.4 is released
	Vector<6> state;
	state[0] = sd_ned_.velocity[0] - params.rU.value;
	state[1] = sd_ned_.velocity[1] - params.rW.value;
	state[2] = sd_ned_.angularRate[1];
	state[3] = sd_ned_.attitude[1] - params.rP.value;// - params.rP.value;
	state[4] = Ep_;
	state[5] = Ev_;

	Vector<2> stateOut = -k_ * state;

	// + delta E -> elevator down, right hand rule behind
	stateOut[0] = stateOut[0] * -1;

//	std::cout << "du: " << state[0] << "cmd: " << -stateOut[1] << "Integral: " << state[5] << "Diff: " << target_.climbAngle - sd_ned_.attitude[1] << "\n";
//	std::cout << "dtheta: " << state[3] << "cmd: " << stateOut[0] << "Integral: " << state[4] << "Diff: " << target_.velocity - sd_ned_.velocity[0] << "\n";
	std::printf("du: %2.8lf cmd: %2.8lf int: %2.8lf diff: %2.8lf\n", state[0], stateOut[1], state[5], target_.velocity - sd_ned_.velocity[0]);
	std::printf("dθ: %2.8lf cmd: %2.8lf int: %2.8lf diff: %2.8lf\n", radToDeg(state[3]), stateOut[0], state[4], radToDeg(target_.climbAngle - sd_ned_.attitude[1]));

	output_.pitchOutput = std::clamp(stateOut[0] + params.tE.value, (FloatingType) -1, (FloatingType) 1);
	output_.throttleOutput = std::clamp(stateOut[1] + params.tT.value, (FloatingType) -1, (FloatingType) 1);
//	output_.pitchOutput = params.tE.value;
//	output_.throttleOutput = params.tT.value;
}

bool
PitchStateSpaceCascade::configure(const Configuration& config)
{
	configure_(config.get_child("cascade"));

	auto retVal = ConfigurableObject::configure(config);


//	a_ = decltype(a_)(params.a.value.data());
//	b_ = decltype(b_)(params.b.value.data());
	k_ = decltype(k_)(params.k.value.data());
	return retVal;
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
//	ParameterRef<decltype(pitchConstraint_.operator*())> pitchConstraint(*pitchConstraint_, "pitch_constraint", true);

	c & rollConstraint;
	c & rollRateConstraint;
//	c & pitchConstraint;

}

void
PitchStateSpaceCascade::setManeuverOverride(const Override& override)
{
	CPSLOG_WARN << "Not implemented";
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
