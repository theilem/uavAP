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
 * HelicopterSimplePIDCascade.cpp
 *
 *  Created on: Sep 1, 2017
 *      Author: simonyu
 */

#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDMapping.h"
#include "uavAP/FlightControl/Controller/PIDController/SimplePIDController/detail/HelicopterSimplePIDCascade.h"

HelicopterSimplePIDCascade::HelicopterSimplePIDCascade(SensorData* sensorData, ControllerTarget* target,
		ControllerOutput* output) :
		controlEnv_(&sensorData->timestamp), hardRollConstraint_(30.0), hardPitchConstraint_(30.0), climbRateOffset_(
				1.0), throttleTarget_(1.0)
{
	APLOG_TRACE << "Create HelicopterCascade";

	Control::PID::Parameters defaultParams;
	defaultParams.kp = 1;

	/* Velocity Y Control */
	auto velocityYInput = controlEnv_.addInput(&sensorData->velocity[1]); // Load Velocity Y Input
	auto accelerationYInput = controlEnv_.addInput(&sensorData->acceleration[1]); // Load Acceleration Y Input
	auto velocityYTarget = controlEnv_.addInput(&target->velocity[1]); // Load Velocity Y Target
	velocityYPID_ = controlEnv_.addPID(velocityYTarget, velocityYInput, accelerationYInput,
			defaultParams); // Compute Velocity Y Output

	/* Roll Control */
	auto rollInput = controlEnv_.addInput(&sensorData->attitude[0]); // Load Roll Input
	auto rollRateInput = controlEnv_.addInput(&sensorData->angularRate[0]); // Load Roll Rate X Input
	rollConstraint_ = controlEnv_.addConstraint(velocityYPID_, -hardRollConstraint_ * M_PI / 180.0,
			hardRollConstraint_ * M_PI / 180.0); // Constraint Roll Target
	rollPID_ = controlEnv_.addPID(rollConstraint_, rollInput, rollRateInput, defaultParams); // Compute Roll Output

	/* Roll Output */
	auto rollOutput = controlEnv_.addConstraint(rollPID_, -1, 1); // Constraint Roll Output
	controlEnv_.addOutput(rollOutput, &output->rollOutput); // Store Roll Output as Controller Roll Output

	/* Velocity X Control */
	auto velocityXInput = controlEnv_.addInput(&sensorData->groundSpeed); // Load Velocity X Input
	auto accelerationXInput = controlEnv_.addInput(&sensorData->acceleration[0]); // Load Acceleration X Input
	auto velocityXTarget = controlEnv_.addInput(&target->velocity[0]); // Load Velocity X Target
	velocityXPID_ = controlEnv_.addPID(velocityXTarget, velocityXInput, accelerationXInput,
			defaultParams); // Compute Velocity X Output

	/* Pitch Control */
	auto pitchInput = controlEnv_.addInput(&sensorData->attitude[1]); // Load Pitch Input
	auto pitchRateInput = controlEnv_.addInput(&sensorData->angularRate[1]); // Load Pitch Rate Input
	pitchConstraint_ = controlEnv_.addConstraint(velocityXPID_,
			-hardPitchConstraint_ * M_PI / 180.0, hardPitchConstraint_ * M_PI / 180.0); // Constraint Pitch Target
	pitchPID_ = controlEnv_.addPID(pitchConstraint_, pitchInput, pitchRateInput, defaultParams); // Compute Pitch Output

	/* Pitch Output */
	auto pitchOutput = controlEnv_.addConstraint(pitchPID_, -1, 1); // Constraint Pitch Output
	controlEnv_.addOutput(pitchOutput, &output->pitchOutput); // Store Pitch Output as Controller Pitch Output

	/* Yaw Rate Control */
	auto yawRateInput = controlEnv_.addInput(&sensorData->angularRate[2]); // Load Yaw Rate Input
	auto yawAccInput = controlEnv_.addInput(&sensorData->angularAcc[2]); // Load Yaw Acceleration Input
	auto yawRateTarget = controlEnv_.addInput(&target->yawRate); // Load Yaw Rate Target
	yawRatePID_ = controlEnv_.addPID(yawRateTarget, yawRateInput, yawAccInput, defaultParams); // Compute Yaw Rate Output

	/* Yaw Rate Output */
	auto yawRateOutput = controlEnv_.addConstraint(yawRatePID_, -1, 1); // Constraint Yaw Rate Output
	controlEnv_.addOutput(yawRateOutput, &output->yawOutput); // Store Yaw Rate Output as Controller Yaw Output

	/* Climb Rate Control */
	auto climbRateInput = controlEnv_.addInput(&sensorData->velocity[2]); // Load Climb Rate Input
	auto accelerationInput = controlEnv_.addInput(&sensorData->acceleration[2]); // Load Climb Acceleration Input
	auto climbRateTarget = controlEnv_.addInput(&target->velocity[2]); // Load Climb Rate Target
	climbRatePID_ = controlEnv_.addPID(climbRateTarget, climbRateInput, accelerationInput,
			defaultParams); // Compute Climb Rate Output

	/* Collective Output */
	auto climbRateOffset = controlEnv_.addConstant(climbRateOffset_); // Load Collective Offset
	auto climbRateOffsetOutput = controlEnv_.addSum(climbRatePID_, climbRateOffset); // Offset Climb Rate Output
	auto climbRateOutput = controlEnv_.addConstraint(climbRateOffsetOutput, -1, 1); // Constraint Climb Rate Output
	controlEnv_.addOutput(climbRateOutput, &output->collectiveOutput); // Store Climb Rate Output as Controller Collective Output

	/* Throttle Output */
	auto throttleOutput = controlEnv_.addConstant(throttleTarget_);
	controlEnv_.addOutput(throttleOutput, &output->throttleOutput);
}

bool
HelicopterSimplePIDCascade::configure(const Configuration& config)
{
	PropertyMapper pm(config);
	pm.add<double>("hard_roll_constraint", hardRollConstraint_, false);
	pm.add<double>("hard_pitch_constraint", hardPitchConstraint_, false);
	pm.add<double>("climb_rate_offset", climbRateOffset_, false);
	pm.add<double>("throttle_target", throttleTarget_, false);

	boost::property_tree::ptree pidConfig;
	pm.add("pids", pidConfig, false);

	rollConstraint_->setContraintValue(hardRollConstraint_ * M_PI / 180.0);
	pitchConstraint_->setContraintValue(hardPitchConstraint_ * M_PI / 180.0);

	Control::PID::Parameters params;
	for (auto it : pidConfig)
	{
		if (!params.configure(it.second))
		{
			APLOG_ERROR << it.first << " configuration not valid.";
			continue;
		}

		tunePID((int) EnumMap<PIDs>::convert(it.first), params);
	}
	return true;
}

bool
HelicopterSimplePIDCascade::tunePID(int pidIndicator, const Control::PID::Parameters& params)
{
	PIDs pid = static_cast<PIDs>(pidIndicator);

	switch (pid)
	{
	case PIDs::VELOCITY_X:
		velocityXPID_->setControlParameters(params);
		break;
	case PIDs::VELOCITY_Y:
		velocityYPID_->setControlParameters(params);
		break;
	case PIDs::CLIMB_RATE:
		climbRatePID_->setControlParameters(params);
		break;
	case PIDs::ROLL:
		rollPID_->setControlParameters(params);
		break;
	case PIDs::PITCH:
		pitchPID_->setControlParameters(params);
		break;
	case PIDs::YAW_RATE:
		yawRatePID_->setControlParameters(params);
		break;
	default:
		APLOG_WARN << "Unknown pidIndicator. Ignore.";
		return false;
	}
	return true;
}

bool
HelicopterSimplePIDCascade::tuneRollBounds(double min, double max)
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
	rollConstraint_->setContraintValue(min / 180. * M_PI, max / 180. * M_PI);
	return true;
}

bool
HelicopterSimplePIDCascade::tunePitchBounds(double min, double max)
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
	pitchConstraint_->setContraintValue(min / 180. * M_PI, max / 180. * M_PI);
	return true;
}

void
HelicopterSimplePIDCascade::evaluate()
{
	controlEnv_.evaluate();
}

std::map<int, PIDStatus>
HelicopterSimplePIDCascade::getPIDStatus()
{
	std::map<int, PIDStatus> status;
	status.insert(std::make_pair((int) PIDs::VELOCITY_X, velocityXPID_->getStatus()));
	status.insert(std::make_pair((int) PIDs::VELOCITY_Y, velocityYPID_->getStatus()));
	status.insert(std::make_pair((int) PIDs::CLIMB_RATE, climbRatePID_->getStatus()));
	status.insert(std::make_pair((int) PIDs::ROLL, rollPID_->getStatus()));
	status.insert(std::make_pair((int) PIDs::PITCH, pitchPID_->getStatus()));
	status.insert(std::make_pair((int) PIDs::YAW_RATE, yawRatePID_->getStatus()));
	return status;
}
