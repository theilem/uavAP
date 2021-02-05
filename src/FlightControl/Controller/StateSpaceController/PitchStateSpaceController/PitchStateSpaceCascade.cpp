//
// Created by seedship on 1/30/21.
//


#include "uavAP/FlightControl/Controller/StateSpaceController/PitchStateSpaceCascade.h"
#include <uavAP/FlightControl/Controller/ControllerTarget.h>

PitchStateSpaceCascade::PitchStateSpaceCascade(const SensorData& sd_enu, const SensorData& sd_ned, const ControllerTarget & target, ControllerOutput& output):
		controlEnv_(&sd_enu.timestamp), sd_ned_(sd_ned), output_(output)//, a_(a.data()), b_(b.data()), k_(k.data())
{
	/* Roll Control */
	auto yawrateTarget = controlEnv_.addInput(&target.yawRate);
	auto airspeed = controlEnv_.addInput(&sd_enu.airSpeed);

	auto rollCalc = std::make_shared<Control::CustomFunction2>(yawrateTarget, airspeed, [](FloatingType yawrate, FloatingType airspeed) {return -std::atan2(airspeed * yawrate, 9.81);});

	rollConstraint_ = std::make_shared<Control::Constraint<Angle<FloatingType>>>(rollCalc);

	auto rollInput = controlEnv_.addInput(&sd_enu.attitude[1]);
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
	auto cmdPitch = controlEnv_.addInput(&target.climbAngle);
	auto pitch = controlEnv_.addInput(&sd_enu.attitude[1]);
	auto pitchConstrained = controlEnv_.addConstraint(pitch, -1, 1);
	pitchConstraint_ = std::make_shared<Control::Constraint<Angle<FloatingType>>>(pitchConstrained);

	auto ep = controlEnv_.addDifference(cmdPitch, pitchConstrained);
	auto cEp = controlEnv_.addIntegrator(ep);
	controlEnv_.addOutput(cEp, &Ep_);

	auto cmdVelocity = controlEnv_.addInput(&target.velocity);
	auto velocity = controlEnv_.addInput(&sd_ned_.velocity[0]);

	auto ev = controlEnv_.addDifference(cmdVelocity, velocity);
	auto cEv = controlEnv_.addIntegrator(ev);
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
	return false;
}

bool
PitchStateSpaceCascade::tunePitchBounds(FloatingType min, FloatingType max)
{
	return false;
}

std::map<PIDs, PIDStatus>
PitchStateSpaceCascade::getPIDStatus()
{
	return std::map<PIDs, PIDStatus>();
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
	state[1] = sd_ned_.velocity[1];
	state[2] = sd_ned_.angularRate[1];
	state[3] = sd_ned_.attitude[1] - params.rP.value;
	state[4] = Ep_;
	state[5] = Ev_;

	auto stateOut = k_ * state;

	output_.pitchOutput = std::clamp(stateOut[0] + params.tE.value, (FloatingType) -1, (FloatingType) 1);
	output_.throttleOutput = std::clamp(stateOut[1] + params.tT.value, (FloatingType) -1, (FloatingType) 1);
}

bool
PitchStateSpaceCascade::configure(const Configuration& config)
{
	 PropertyMapper<Configuration> c(config);

	auto retVal = ConfigurableObject::configure(config);

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

	c & rollConstraint;
	c & rollRateConstraint;
	c & pitchConstraint;

//	a_ = decltype(a_)(params.a.value.data());
//	b_ = decltype(b_)(params.b.value.data());
	k_ = decltype(k_)(params.k.value.data());
	return retVal;
}

void
PitchStateSpaceCascade::printGains()
{
//	CPSLOG_WARN << a_;
//	CPSLOG_WARN << b_;
	CPSLOG_WARN << k_;
}