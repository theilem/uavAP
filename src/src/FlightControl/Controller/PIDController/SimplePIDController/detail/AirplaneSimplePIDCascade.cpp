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
 * AirplaneSimplePIDCascade.cpp
 *
 *  Created on: Aug 14, 2017
 *      Author: mircot
 */

#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDMapping.h"
#include "uavAP/FlightControl/Controller/PIDController/SimplePIDController/detail/AirplaneSimplePIDCascade.h"

AirplaneSimplePIDCascade::AirplaneSimplePIDCascade(SensorData* sensorData, Vector3& velInertial,
		Vector3& accInertial, ControllerTarget* target, ControllerOutput* output) :
		controlEnv_(&sensorData->timestamp), hardRollConstraint_(30.0), hardPitchConstraint_(30.0)
{
	APLOG_TRACE << "Create AirplaneCascade";

	Control::PIDParameters defaultParams;

	/* Yaw Rate Control */
	auto yawRateInput = controlEnv_.addInput(&sensorData->angularRate[2]);
	//auto yawAccInput = controlEnv_.addInput(&sensorData->angularAcc[2]);
	auto yawRateTarget = controlEnv_.addInput(&target->yawRate);
	//yawRatePID_ = controlEnv_.addPID(yawRateTarget, yawRateInput, yawAccInput, defaultParams);
	yawRatePID_ = controlEnv_.addPID(yawRateTarget, yawRateInput, defaultParams);
	rollConstraint_ = controlEnv_.addConstraint(yawRatePID_, -hardRollConstraint_ * M_PI / 180.0,
			hardRollConstraint_ * M_PI / 180.0);

	/* Roll Control */
	auto rollInput = controlEnv_.addInput(&sensorData->attitude[0]);
	auto rollRateInput = controlEnv_.addInput(&sensorData->angularRate[0]);
	rollPID_ = controlEnv_.addPID(rollConstraint_, rollInput, rollRateInput, defaultParams);

	/* Roll Output */
	auto rollOutConstraint = controlEnv_.addConstraint(rollPID_, -1, 1);
	controlEnv_.addOutput(rollOutConstraint, &output->rollOutput);

	/* Climb Rate Output */
	auto climbRateInput = controlEnv_.addInput(&velInertial[2]);
	auto climbAccInput = controlEnv_.addInput(&accInertial[2]);
	auto climbRateTarget = controlEnv_.addInput(&target->climbAngle);
	climbRatePID_ = controlEnv_.addPID(climbRateTarget, climbRateInput, climbAccInput,
			defaultParams);
	pitchConstraint_ = controlEnv_.addConstraint(climbRatePID_,
			-hardPitchConstraint_ * M_PI / 180.0, hardPitchConstraint_ * M_PI / 180.0);

	/* Pitch Control */
	auto pitchInput = controlEnv_.addInput(&sensorData->attitude[1]);
	auto pitchRateInput = controlEnv_.addInput(&sensorData->angularRate[1]);
	pitchPID_ = controlEnv_.addPID(pitchConstraint_, pitchInput, pitchRateInput, defaultParams);

	/* Pitch Output */
	auto pitchOutConstraint = controlEnv_.addConstraint(pitchPID_, -1, 1);
	controlEnv_.addOutput(pitchOutConstraint, &output->pitchOutput);

	/* Velocity Control */
	auto velocityInput = controlEnv_.addInput(&sensorData->velocity[0]);
	auto accelerationInput = controlEnv_.addInput(&sensorData->acceleration[0]);
	auto velocityTarget = controlEnv_.addInput(&target->velocity);
	velocityPID_ = controlEnv_.addPID(velocityTarget, velocityInput, accelerationInput,
			defaultParams);

	/* Throttle Output */
	auto velocityOffset = controlEnv_.addConstant(1);
	auto velocityDifference = controlEnv_.addDifference(velocityPID_, velocityOffset);
	auto velocityConstraint = controlEnv_.addConstraint(velocityDifference, -1, 1);
	controlEnv_.addOutput(velocityConstraint, &output->throttleOutput);

}

bool
AirplaneSimplePIDCascade::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);
	pm.add<FloatingType>("hard_roll_constraint", hardRollConstraint_, false);
	pm.add<FloatingType>("hard_pitch_constraint", hardPitchConstraint_, false);

	Configuration pidConfig;
	pm.add("pids", pidConfig, false);

	rollConstraint_->setConstraintValue(hardRollConstraint_ * M_PI / 180.0);
	pitchConstraint_->setConstraintValue(hardPitchConstraint_ * M_PI / 180.0);

	Control::PIDParameters params;
	for (const auto& it : pidConfig)
	{
		auto pid = EnumMap<PIDs>::convert(it.first);

		switch (pid)
		{
		case PIDs::CLIMB_ANGLE:
			climbRatePID_->configure(it.second);
			break;
		case PIDs::PITCH:
			pitchPID_->configure(it.second);
			break;
		case PIDs::YAW_RATE:
			yawRatePID_->configure(it.second);
			break;
		case PIDs::ROLL:
			rollPID_->configure(it.second);
			break;
		case PIDs::VELOCITY:
			velocityPID_->configure(it.second);
			break;
		default:
			APLOG_WARN << "Unknown pidIndicator. Ignore.";
		}

	}
	return true;
}

bool
AirplaneSimplePIDCascade::tunePID(PIDs pid, const Control::PIDParameters& params)
{
	switch (pid)
	{
	case PIDs::CLIMB_ANGLE:
		climbRatePID_->setParams(params);
		break;
	case PIDs::PITCH:
		pitchPID_->setParams(params);
		break;
	case PIDs::YAW_RATE:
		yawRatePID_->setParams(params);
		break;
	case PIDs::ROLL:
		rollPID_->setParams(params);
		break;
	case PIDs::VELOCITY:
		velocityPID_->setParams(params);
		break;
	default:
		APLOG_WARN << "Unknown pidIndicator. Ignore.";
		return false;
	}
	return true;
}

bool
AirplaneSimplePIDCascade::tuneRollBounds(FloatingType min, FloatingType max)
{
	if ((min < -hardRollConstraint_) || (min > 0.0))
	{
		APLOG_WARN << "Roll constraint min violates hard constraint.";
		return false;
	}
	if ((max > hardRollConstraint_) || (max < 0.0))
	{
		APLOG_WARN << "Roll constraint max violates hard constraint.";
		return false;
	}
	rollConstraint_->setConstraintValue(min / 180. * M_PI, max / 180. * M_PI);
	return true;
}

bool
AirplaneSimplePIDCascade::tunePitchBounds(FloatingType min, FloatingType max)
{
	if ((min < -hardPitchConstraint_) || (min > 0.0))
	{
		APLOG_WARN << "Pitch constraint min violates hard constraint.";
		return false;
	}
	if ((max > hardPitchConstraint_) || (max < 0.0))
	{
		APLOG_WARN << "Pitch constraint max violates hard constraint.";
		return false;
	}
	pitchConstraint_->setConstraintValue(min / 180. * M_PI, max / 180. * M_PI);
	return true;
}

void
AirplaneSimplePIDCascade::evaluate()
{
	controlEnv_.evaluate();
}

std::map<PIDs, PIDStatus>
AirplaneSimplePIDCascade::getPIDStatus()
{
	std::map<PIDs, PIDStatus> status;
	status.insert(std::make_pair(PIDs::CLIMB_ANGLE, climbRatePID_->getStatus()));
	status.insert(std::make_pair(PIDs::PITCH, pitchPID_->getStatus()));
	status.insert(std::make_pair(PIDs::ROLL, rollPID_->getStatus()));
	status.insert(std::make_pair(PIDs::VELOCITY, velocityPID_->getStatus()));
	status.insert(std::make_pair(PIDs::YAW_RATE, yawRatePID_->getStatus()));
	return status;
}
