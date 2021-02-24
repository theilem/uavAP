/*
 * ManeuverRateCascade.cpp
 *
 *  Created on: Oct 10, 2019
 *      Author: mirco
 */
#include <uavAP/Core/SensorData.h>
#include <uavAP/FlightControl/Controller/ControlElements/ControlElements.h>
#include <uavAP/FlightControl/Controller/ControlElements/EvaluableControlElements.h>
#include <uavAP/FlightControl/Controller/ControllerTarget.h>
#include <uavAP/FlightControl/Controller/PIDController/PIDHandling.h>
#include <uavAP/FlightControl/Controller/PIDController/ManeuverRatePIDController/ManeuverRateCascade.h>
#include <uavAP/MissionControl/ManeuverPlanner/Override.h>
#include <cmath>

ManeuverRateCascade::ManeuverRateCascade(const SensorData& sd, const ControllerTarget& target,
										 ControllerOutput& out) :
		controlEnv_(&sd.timestamp), throttleOverrideValue_(0)
{

	/* Roll Control */
	auto yawrateTarget = controlEnv_.addInput(&target.yawRate);
	auto airspeed = controlEnv_.addInput(&sd.airSpeed);

	auto rollCalc = std::make_shared<Control::CustomFunction2>(yawrateTarget, airspeed,
															   std::bind(&ManeuverRateCascade::yawrateToRoll, this,
																		 std::placeholders::_1,
																		 std::placeholders::_2));

	rollConstraint_ = std::make_shared<Control::Constraint<Angle<FloatingType>>>(rollCalc);

	auto rollInput = controlEnv_.addInput(&sd.attitude[0]);
	// ENU - QPR
	auto rollRateInput = controlEnv_.addInput(&sd.angularRate[1]);

	Control::PIDParameters defaultParams;
	auto rollPID = controlEnv_.addPID(rollConstraint_, rollInput, rollRateInput, defaultParams);
	rollRateTargetConstraint_ = std::make_shared<Control::Constraint<Angle<FloatingType>>>(rollPID);

	/* Roll Rate Control */
	auto rollRatePID = controlEnv_.addPID(rollRateTargetConstraint_, rollRateInput, defaultParams);

	/* Roll Output */

	auto rollOutConstraint = controlEnv_.addConstraint(rollRatePID, -1, 1);

	auto rollOut = controlEnv_.addOutput(rollOutConstraint, &out.rollOutput);

	/* Climb Angle Control*/
	auto aoaInput = controlEnv_.addInput(&sd.angleOfAttack);
	auto pitchInput = controlEnv_.addInput(&sd.attitude[1]);

	auto climbAngle = controlEnv_.addSum(pitchInput, aoaInput);

	auto climbAngleTarget = controlEnv_.addInput(&target.climbAngle);

	auto climbAnglePID = controlEnv_.addPID(climbAngleTarget, climbAngle, defaultParams);
	pitchConstraint_ = std::make_shared<Control::Constraint<Angle<FloatingType>>>(climbAnglePID);

	/* Pitch Control */
	auto pitchRateInput = controlEnv_.addInput(&sd.angularRate[0]);

	auto pitchPID = controlEnv_.addPID(pitchConstraint_, pitchInput, pitchRateInput, defaultParams);

	/* Pitch Output */
	auto pitchOutConstraint = controlEnv_.addConstraint(pitchPID, -1, 1);

	auto pitchOut = controlEnv_.addOutput(pitchOutConstraint, &out.pitchOutput);

	/* Velocity Control */
	auto accelerationInput = controlEnv_.addInput(&sd.acceleration[0]);
	auto velocityTarget = controlEnv_.addInput(&target.velocity);

	auto velocityPID = controlEnv_.addPID(velocityTarget, airspeed, accelerationInput,
										  defaultParams);

	/* Throttle Output */
	auto velocityOffset = controlEnv_.addConstant(1);
	auto velocityDifference = controlEnv_.addDifference(velocityPID, velocityOffset);

	throttleConstraint_ = controlEnv_.addConstraint(velocityDifference, -1, 1);
	auto throttleOverride = controlEnv_.addInput(&throttleOverrideValue_);
	throttleSwitch_ = controlEnv_.addManualSwitch(throttleOverride, throttleConstraint_);
	throttleSwitch_->switchTo(0);

	auto throttleOut = controlEnv_.addOutput(throttleSwitch_, &out.throttleOutput);




	/* Rudder Output */
//	auto rudderBeta = controlEnv_.addInput(&beta_);
//	auto rudderTarget = controlEnv_.addConstant(0);
//
//	auto rudderPID = controlEnv_.addPID(rudderTarget, rudderBeta, defaultParams);
//
//	auto invertedRudder = controlEnv_.addGain(rudderPID, -1);
//
//	auto constraintYawOut = controlEnv_.addConstraint(invertedRudder, -1, 1);
//
//	auto yawOut = controlEnv_.addOutput(constraintYawOut, &output->yawOutput);
//	pids_.insert(std::make_pair(PIDs::RUDDER, rudderPID));
	pids_.insert(std::make_pair(PIDs::VELOCITY, velocityPID));
	pids_.insert(std::make_pair(PIDs::PITCH, pitchPID));
	pids_.insert(std::make_pair(PIDs::CLIMB_ANGLE, climbAnglePID));
	pids_.insert(std::make_pair(PIDs::ROLL, rollPID));
	pids_.insert(std::make_pair(PIDs::ROLL_RATE, rollRatePID));

}

bool
ManeuverRateCascade::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);

	configureParams(pm);
	if (throttleOverride_.has_value())
	{
		setThrottleOverride(throttleOverride_.value());
	}

	return pm.map();
}

bool
ManeuverRateCascade::tunePID(PIDs pid, const Control::PIDParameters& params)
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
ManeuverRateCascade::tuneRollBounds(double min, double max)
{
	CPSLOG_ERROR << "Deprecated function called";
	return false;
}

bool
ManeuverRateCascade::tunePitchBounds(double min, double max)
{
	CPSLOG_ERROR << "Deprecated function called";
	return false;
}

std::map<PIDs, PIDStatus>
ManeuverRateCascade::getPIDStatus()
{
	std::map<PIDs, PIDStatus> stati;
	for (const auto& pid : pids_)
	{
		stati.insert(std::make_pair(pid.first, pid.second->getStatus()));
	}
	return stati;
}

void
ManeuverRateCascade::evaluate()
{
	controlEnv_.evaluate();
}

FloatingType
ManeuverRateCascade::yawrateToRoll(FloatingType yawrate, FloatingType airspeed)
{
	return -std::atan2(airspeed * yawrate, 9.81);
}

void
ManeuverRateCascade::setManeuverOverride(const Override& override)
{
	for (auto& it : pids_)
		it.second->disableOverride();

	for (const auto& it : override.pid)
	{
		if (auto pid = findInMap(pids_, it.first))
			pid->second->overrideTarget(it.second);
	}

}

Optional<PIDParams>
ManeuverRateCascade::getPIDParams(const DataRequest& request)
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

ManeuverRateCascade::ManeuverRateCascade() : controlEnv_(nullptr)
{

}

void
ManeuverRateCascade::setThrottleLimit(FloatingType maxThrottle)
{
	throttleConstraint_->setConstraintValue(-1, maxThrottle);
}

void
ManeuverRateCascade::setThrottleOverride(FloatingType throttleOverride)
{
	throttleOverrideValue_ = throttleOverride;
	throttleSwitch_->switchTo(1);
}

void
ManeuverRateCascade::disableThrottleOverride()
{
	throttleSwitch_->switchTo(0);
	throttleOverride_.reset();
}