////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/*
 * ManeuverCascade.cpp
 *
 *  Created on: Sep 15, 2017
 *      Author: mircot
 */

#include <map>

#include "uavAP/Core/SensorData.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/Controller/PIDController/RatePIDController/detail/RateCascade.h"

RateCascade::RateCascade(SensorData* sensorData, Vector3& velInertial, Vector3& accInertial,
		ControllerTarget* target, ControllerOutput* output, ServoData* servoData) :
		sensorData_(sensorData), controllerTarget_(target), controlEnv_(&sensorData->timestamp), hardRollConstraint_(
				30.0), hardRollRateConstraint_(30.0), hardPitchConstraint_(30.0), hardPitchRateConstraint_(
				30.0), rollConstraint_(30.0), rollRateConstraint_(30.0), rollOutConstraint_(1.0), pitchConstraint_(
				30.0), pitchRateConstraint_(30.0), pitchOutConstraint_(1.0), yawOutConstraint_(1.0), throttleOutConstraint_(
				1.0), rollTarget_(0), useRPMController_(false)
{
	CPSLOG_TRACE << "Create RateCascade";

	Control::PIDParameters defaultParams;

	/* Roll Control */
	auto rollTarget = controlEnv_.addInput(&rollTarget_);
	auto rollInput = controlEnv_.addInput(&sensorData->attitude[0]);
	auto rollRateInput = controlEnv_.addInput(&sensorData->angularRate[0]);
	rollTargetConstraint_ = controlEnv_.addConstraint(rollTarget, degToRad(-rollConstraint_),
			degToRad(rollConstraint_), degToRad(-hardRollConstraint_),
			degToRad(hardRollConstraint_));
	auto rollPID = controlEnv_.addPID(rollTargetConstraint_, rollInput, rollRateInput,
			defaultParams);
	rollRateTargetConstraint_ = controlEnv_.addConstraint(rollPID, degToRad(-rollRateConstraint_),
			degToRad(rollRateConstraint_), degToRad(-hardRollRateConstraint_),
			degToRad(hardRollRateConstraint_));

	/* Roll Rate Control */
	auto rollRatePID = controlEnv_.addPID(rollRateTargetConstraint_, rollRateInput, defaultParams);

	/* Roll Output */
	rollOutputConstraint_ = controlEnv_.addConstraint(rollRatePID, -1, 1);
	auto rollOut = controlEnv_.addOutput(rollOutputConstraint_, &output->rollOutput);

	/* Climb Angle Control*/
	auto aoaInput = controlEnv_.addInput(&sensorData->angleOfAttack);
	auto pitchInput = controlEnv_.addInput(&sensorData->attitude[1]);
	auto climbAngle = controlEnv_.addDifference(pitchInput, aoaInput);
	auto climbAngleTarget = controlEnv_.addInput(&target->climbAngle);
	auto climbAnglePID = controlEnv_.addPID(climbAngleTarget, climbAngle, defaultParams);
	pitchTargetConstraint_ = controlEnv_.addConstraint(climbAnglePID, degToRad(-pitchConstraint_),
			degToRad(pitchConstraint_), degToRad(-hardPitchConstraint_),
			degToRad(hardPitchConstraint_));

	/* Pitch Control */
	auto pitchRateInput = controlEnv_.addInput(&sensorData->angularRate[1]);
	auto pitchPID = controlEnv_.addPID(pitchTargetConstraint_, pitchInput, pitchRateInput,
			defaultParams);
	pitchRateTargetConstraint_ = controlEnv_.addConstraint(pitchPID,
			degToRad(-pitchRateConstraint_), degToRad(pitchRateConstraint_),
			degToRad(-hardPitchRateConstraint_), degToRad(hardPitchRateConstraint_));

	/* Pitch Rate Control */
	auto pitchRatePID = controlEnv_.addPID(pitchRateTargetConstraint_, pitchRateInput,
			defaultParams);

	/* Pitch Output */
	pitchOutputConstraint_ = controlEnv_.addConstraint(pitchRatePID, -1, 1);
	auto pitchOut = controlEnv_.addOutput(pitchOutputConstraint_, &output->pitchOutput);

	/* Velocity Control */
	auto velocityInput = controlEnv_.addInput(&sensorData->airSpeed);
	auto accelerationInput = controlEnv_.addInput(&sensorData->acceleration[0]);
	auto velocityTarget = controlEnv_.addInput(&target->velocity);
	auto velocityPID = controlEnv_.addPID(velocityTarget, velocityInput, accelerationInput,
			defaultParams);

	Control::Element throttleTarget = velocityPID;
	if (servoData)
	{
		/* RPM Control */
		auto rpmInput = controlEnv_.addInput(&servoData->rpm);
		auto rpmInputFilter = controlEnv_.addFilter(rpmInput, 0.5);
		auto rpmTarget = controlEnv_.addConstant(0);
		auto rpmPID = controlEnv_.addPID(rpmTarget, rpmInputFilter, defaultParams);

		pids_.insert(std::make_pair(PIDs::RPM, rpmPID));

		throttleManualSwitch_ = controlEnv_.addManualSwitch(rpmPID, velocityPID);
		throttleManualSwitch_->switchTo(useRPMController_);
		throttleTarget = throttleManualSwitch_;
	}
	else
	{
		if (useRPMController_)
		{
			CPSLOG_ERROR << "RPM Controller requested but not available because ServoData is missing";
		}
	}

	/* Throttle Output */
	auto throttleOffset = controlEnv_.addConstant(1);
	auto throttleDifference = controlEnv_.addDifference(throttleTarget, throttleOffset);
	throttleOutputConstraint_ = controlEnv_.addConstraint(throttleDifference, -1, 1);
	auto throttleOut = controlEnv_.addOutput(throttleOutputConstraint_, &output->throttleOutput);

	/* Rudder Output */
	auto rudderBeta = controlEnv_.addInput(&beta_);
	auto rudderTarget = controlEnv_.addConstant(0);
	auto rudderPID = controlEnv_.addPID(rudderTarget, rudderBeta, defaultParams);
	auto invertedRudder = controlEnv_.addGain(rudderPID, -1);

	yawOutputConstraint_ = controlEnv_.addConstraint(invertedRudder, -1, 1);
	auto yawOut = controlEnv_.addOutput(yawOutputConstraint_, &output->yawOutput);

	outputs_.insert(std::make_pair(ControllerOutputs::ROLL, rollOut));
	outputs_.insert(std::make_pair(ControllerOutputs::PITCH, pitchOut));
	outputs_.insert(std::make_pair(ControllerOutputs::THROTTLE, throttleOut));
	outputs_.insert(std::make_pair(ControllerOutputs::YAW, yawOut));

	outputWaveforms_.insert(std::make_pair(ControllerOutputsWaveforms::ROLL, rollOut));
	outputWaveforms_.insert(std::make_pair(ControllerOutputsWaveforms::PITCH, pitchOut));
	outputWaveforms_.insert(std::make_pair(ControllerOutputsWaveforms::THROTTLE, throttleOut));
	outputWaveforms_.insert(std::make_pair(ControllerOutputsWaveforms::YAW, yawOut));

	constraints_.insert(std::make_pair(ControllerConstraints::ROLL, rollTargetConstraint_));
	constraints_.insert(
			std::make_pair(ControllerConstraints::ROLL_RATE, rollRateTargetConstraint_));
	constraints_.insert(std::make_pair(ControllerConstraints::ROLL_OUTPUT, rollOutputConstraint_));
	constraints_.insert(std::make_pair(ControllerConstraints::PITCH, pitchTargetConstraint_));
	constraints_.insert(
			std::make_pair(ControllerConstraints::PITCH_RATE, pitchRateTargetConstraint_));
	constraints_.insert(
			std::make_pair(ControllerConstraints::PITCH_OUTPUT, pitchOutputConstraint_));
	constraints_.insert(std::make_pair(ControllerConstraints::YAW_OUTPUT, yawOutputConstraint_));
	constraints_.insert(
			std::make_pair(ControllerConstraints::THROTTLE_OUTPUT, throttleOutputConstraint_));

	pids_.insert(std::make_pair(PIDs::ROLL, rollPID));
	pids_.insert(std::make_pair(PIDs::ROLL_RATE, rollRatePID));
	pids_.insert(std::make_pair(PIDs::CLIMB_ANGLE, climbAnglePID));
	pids_.insert(std::make_pair(PIDs::PITCH, pitchPID));
	pids_.insert(std::make_pair(PIDs::PITCH_RATE, pitchRatePID));
	pids_.insert(std::make_pair(PIDs::VELOCITY, velocityPID));
	pids_.insert(std::make_pair(PIDs::RUDDER, rudderPID));

}

bool
RateCascade::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);
	pm.add<FloatingType>("hard_roll_constraint", hardRollConstraint_, false);
	pm.add<FloatingType>("hard_pitch_constraint", hardPitchConstraint_, false);
	pm.add<FloatingType>("hard_roll_rate_constraint", hardRollRateConstraint_, false);
	pm.add<FloatingType>("hard_pitch_rate_constraint", hardPitchRateConstraint_, false);

	pm.add<FloatingType>("roll_constraint", rollConstraint_, false);
	pm.add<FloatingType>("pitch_constraint", pitchConstraint_, false);
	pm.add<FloatingType>("roll_rate_constraint", rollRateConstraint_, false);
	pm.add<FloatingType>("pitch_rate_constraint", pitchRateConstraint_, false);

	pm.add<bool>("use_rpm_controller", useRPMController_, false);

	throttleManualSwitch_->switchTo(useRPMController_);

	Configuration pidConfig;
	pm.add("pids", pidConfig, false);

	rollTargetConstraint_->setConstraintValue(degToRad(rollConstraint_));
	pitchTargetConstraint_->setConstraintValue(degToRad(pitchConstraint_));
	rollRateTargetConstraint_->setConstraintValue(degToRad(rollRateConstraint_));
	pitchRateTargetConstraint_->setConstraintValue(degToRad(pitchRateConstraint_));

	for (const auto& it : pidConfig)
	{
		auto pid = pids_.find(EnumMap<PIDs>::convert(it.first));

		if (pid == pids_.end())
		{
			CPSLOG_ERROR << "Unknown pidIndicator. Ignore";
			continue;
		}

		pid->second->configure(it.second);
	}
	return true;
}

bool
RateCascade::tunePID(PIDs pid, const Control::PIDParameters& params)
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
RateCascade::tuneRollBounds(FloatingType min, FloatingType max)
{
	if ((min < -hardRollConstraint_) || (min > 0.0))
	{
		CPSLOG_WARN << "Roll constraint min violates hard constraint.";
		return false;
	}
	if ((max > hardRollConstraint_) || (max < 0.0))
	{
		CPSLOG_WARN << "Roll constraint max violates hard constraint.";
		return false;
	}
	rollTargetConstraint_->setConstraintValue(min / 180. * M_PI, max / 180. * M_PI);
	return true;
}

bool
RateCascade::tunePitchBounds(FloatingType min, FloatingType max)
{
	if ((min < -hardPitchConstraint_) || (min > 0.0))
	{
		CPSLOG_WARN << "Pitch constraint min violates hard constraint.";
		return false;
	}
	if ((max > hardPitchConstraint_) || (max < 0.0))
	{
		CPSLOG_WARN << "Pitch constraint max violates hard constraint.";
		return false;
	}
	pitchTargetConstraint_->setConstraintValue(min / 180. * M_PI, max / 180. * M_PI);
	return true;
}

bool
RateCascade::tuneRollRateBounds(FloatingType min, FloatingType max)
{
	if ((min < -hardRollRateConstraint_) || (min > 0.0))
	{
		CPSLOG_WARN << "Roll rate constraint min violates hard constraint.";
		return false;
	}
	if ((max > hardRollRateConstraint_) || (max < 0.0))
	{
		CPSLOG_WARN << "Roll rate constraint max violates hard constraint.";
		return false;
	}
	rollRateTargetConstraint_->setConstraintValue(min / 180. * M_PI, max / 180. * M_PI);
	return true;
}

bool
RateCascade::tunePitchRateBounds(FloatingType min, FloatingType max)
{
	if ((min < -hardPitchRateConstraint_) || (min > 0.0))
	{
		CPSLOG_WARN << "Pitch rate constraint min violates hard constraint.";
		return false;
	}
	if ((max > hardPitchRateConstraint_) || (max < 0.0))
	{
		CPSLOG_WARN << "Pitch rate constraint max violates hard constraint.";
		return false;
	}
	pitchRateTargetConstraint_->setConstraintValue(min / 180. * M_PI, max / 180. * M_PI);
	return true;
}

PIDStati
RateCascade::getPIDStatus() const
{
	std::map<PIDs, PIDStatus> status;
	for (const auto& it : pids_)
	{
		status.insert(std::make_pair(it.first, it.second->getStatus()));
	}
	return status;
}

void
RateCascade::evaluate()
{
	Vector3 velocityBody;
	FloatingType yaw = sensorData_->attitude.z();
	FloatingType roll = sensorData_->attitude.x();
	FloatingType pitch = -sensorData_->attitude.y();

	Matrix3 m;
	m = AngleAxis(-roll, Vector3::UnitX()) * AngleAxis(-pitch, Vector3::UnitY())
			* AngleAxis(-yaw, Vector3::UnitZ());

	velocityBody = m * sensorData_->velocity;

	FloatingType bigV = velocityBody.norm();
	FloatingType smallV = velocityBody[1];

	beta_ = -asin(smallV / bigV);

	rollTarget_ = -atan2(bigV * controllerTarget_->yawRate, 9.81);

	controlEnv_.evaluate();
}
//
//void
//RateCascade::setManeuverOverride(const Override& override)
//{
//	for (auto& it : pids_)
//	{
//		it.second->disableOverride();
//	}
//
//	for (auto& it : outputs_)
//	{
//		it.second->disableOverride();
//	}
//
//	for (auto& it : constraints_)
//	{
//		it.second->disableOverride();
//	}
//
//	if (override.isEmpty())
//	{
//		return;
//	}
//
//	for (const auto& it : override.pid)
//	{
//		if (auto pid = findInMap(pids_, it.first))
//		{
//			pid->second->overrideTarget(it.second);
//		}
//	}
//
//	for (const auto& it : override.waveform)
//	{
//		if (auto out = findInMap(outputWaveforms_, it.first))
//		{
//			out->second->setWaveform(it.second);
//			out->second->setWavelength(override.wavelength);
//			out->second->setPhase(override.phase);
//		}
//	}
//
//	for (const auto& it : override.output)
//	{
//		if (auto out = findInMap(outputs_, it.first))
//		{
//			out->second->overrideOutput(it.second);
//		}
//	}
//
//	for (const auto& it : override.constraint)
//	{
//		if (auto constraint = findInMap(constraints_, it.first))
//		{
//			constraint->second->overrideConstraintValue(it.second);
//		}
//	}
//}
