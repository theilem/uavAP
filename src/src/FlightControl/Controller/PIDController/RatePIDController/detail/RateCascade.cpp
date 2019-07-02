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

#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/PropertyMapper/ConfigurableObjectImpl.hpp"
#include "uavAP/Core/SensorData.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/Controller/ControllerTarget.h"
#include "uavAP/FlightControl/Controller/PIDController/RatePIDController/detail/RateCascade.h"

RateCascade::RateCascade(SensorData* sensorData, Vector3& velInertial, Vector3& accInertial,
		ControllerTarget* target, ControllerOutput* output) :
		sensorData_(sensorData), controllerTarget_(target), controlEnv_(&sensorData->timestamp), hardRollConstraint_(
				30.0), hardRollRateConstraint_(30.0), hardPitchConstraint_(30.0), hardPitchRateConstraint_(
				30.0), rollTarget_(0)
{
	APLOG_TRACE << "Create RateCascade";

	Control::PIDParameters defaultParams;

	/* Roll Control */
	auto rollTarget = controlEnv_.addInput(&rollTarget_);
	auto rollInput = controlEnv_.addInput(&sensorData->attitude[0]);
	auto rollRateInput = controlEnv_.addInput(&sensorData->angularRate[0]);
	rollTargetConstraint_ = controlEnv_.addConstraint(rollTarget,
				-hardRollConstraint_ * M_PI / 180.0, hardRollConstraint_ * M_PI / 180.0);
	auto rollPID = controlEnv_.addPID(rollTargetConstraint_, rollInput, rollRateInput, defaultParams);
	rollRateTargetConstraint_ = controlEnv_.addConstraint(rollPID,
			-hardRollRateConstraint_ * M_PI / 180.0, hardRollRateConstraint_ * M_PI / 180.0);

	/* Roll Rate Control */
	auto rollRatePID = controlEnv_.addPID(rollRateTargetConstraint_, rollRateInput, defaultParams);

	/* Roll Output */
	auto rollOutputConstraint = controlEnv_.addConstraint(rollRatePID, -1, 1);
	auto rollOut = controlEnv_.addOutput(rollOutputConstraint, &output->rollOutput);

	/* Climb Angle Control*/
	auto aoaInput = controlEnv_.addInput(&sensorData->angleOfAttack);
	auto pitchInput = controlEnv_.addInput(&sensorData->attitude[1]);
	auto climbAngle = controlEnv_.addDifference(pitchInput, aoaInput);
	auto climbAngleTarget = controlEnv_.addInput(&target->climbAngle);
	auto climbAnglePID = controlEnv_.addPID(climbAngleTarget, climbAngle, defaultParams);
	pitchTargetConstraint_ = controlEnv_.addConstraint(climbAnglePID,
			-hardPitchConstraint_ * M_PI / 180.0, hardPitchConstraint_ * M_PI / 180.0);

	/* Pitch Control */
	auto pitchRateInput = controlEnv_.addInput(&sensorData->angularRate[1]);
	auto pitchPID = controlEnv_.addPID(pitchTargetConstraint_, pitchInput, pitchRateInput, defaultParams);
	pitchRateTargetConstraint_ = controlEnv_.addConstraint(pitchPID,
			-hardPitchRateConstraint_ * M_PI / 180.0, hardPitchRateConstraint_ * M_PI / 180.0);

	/* Pitch Rate Control */
	auto pitchRatePID = controlEnv_.addPID(pitchRateTargetConstraint_, pitchRateInput, defaultParams);

	/* Pitch Output */
	auto pitchOutputConstraint = controlEnv_.addConstraint(pitchRatePID, -1, 1);
	auto pitchOut = controlEnv_.addOutput(pitchOutputConstraint, &output->pitchOutput);

	/* Velocity Control */
	auto velocityInput = controlEnv_.addInput(&sensorData->airSpeed);
	auto accelerationInput = controlEnv_.addInput(&sensorData->acceleration[0]);
	auto velocityTarget = controlEnv_.addInput(&target->velocity);
	auto velocityPID = controlEnv_.addPID(velocityTarget, velocityInput, accelerationInput,
			defaultParams);

	/* Throttle Output */
	auto velocityOffset = controlEnv_.addConstant(1);
	auto velocityDifference = controlEnv_.addDifference(velocityPID, velocityOffset);
	auto velocityOutputConstraint = controlEnv_.addConstraint(velocityDifference, -1, 1);
	auto throttleOut = controlEnv_.addOutput(velocityOutputConstraint, &output->throttleOutput);

	/* Rudder Output */
	auto rudderBeta = controlEnv_.addInput(&beta_);
	auto rudderTarget = controlEnv_.addConstant(0);
	auto rudderPID = controlEnv_.addPID(rudderTarget, rudderBeta, defaultParams);
	auto invertedRudder = controlEnv_.addGain(rudderPID, -1);

	auto yawOutputConstraint = controlEnv_.addConstraint(invertedRudder, -1, 1);
	auto yawOut = controlEnv_.addOutput(yawOutputConstraint, &output->yawOutput);

	outputs_.insert(std::make_pair(ControllerOutputs::ROLL, rollOut));
	outputs_.insert(std::make_pair(ControllerOutputs::PITCH, pitchOut));
	outputs_.insert(std::make_pair(ControllerOutputs::THROTTLE, throttleOut));
	outputs_.insert(std::make_pair(ControllerOutputs::YAW, yawOut));


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
	pm.add<double>("hard_roll_constraint", hardRollConstraint_, false);
	pm.add<double>("hard_pitch_constraint", hardPitchConstraint_, false);
	pm.add<double>("hard_roll_rate_constraint", hardRollRateConstraint_, false);
	pm.add<double>("hard_pitch_rate_constraint", hardPitchRateConstraint_, false);

	Configuration pidConfig;
	pm.add("pids", pidConfig, false);

	rollTargetConstraint_->setContraintValue(hardRollConstraint_ * M_PI / 180.0);
	pitchTargetConstraint_->setContraintValue(hardPitchConstraint_ * M_PI / 180.0);
	rollRateTargetConstraint_->setContraintValue(hardRollRateConstraint_ * M_PI / 180.0);
	pitchRateTargetConstraint_->setContraintValue(hardPitchRateConstraint_ * M_PI / 180.0);

	for (const auto& it : pidConfig)
	{
		auto pid = pids_.find(EnumMap<PIDs>::convert(it.first));

		if (pid == pids_.end())
		{
			APLOG_ERROR << "Unknown pidIndicator. Ignore";
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
		APLOG_ERROR << "Unknown pidIndicator. Ignore";
		return false;
	}

	it->second->setParams(params);
	return true;
}

bool
RateCascade::tuneRollBounds(double min, double max)
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
	rollTargetConstraint_->setContraintValue(min / 180. * M_PI, max / 180. * M_PI);
	return true;
}

bool
RateCascade::tunePitchBounds(double min, double max)
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
	pitchTargetConstraint_->setContraintValue(min / 180. * M_PI, max / 180. * M_PI);
	return true;
}

bool
RateCascade::tuneRollRateBounds(double min, double max)
{
	if ((min < -hardRollRateConstraint_) || (min > 0.0))
	{
		APLOG_WARN << "Roll rate constraint min violates hard constraint.";
		return false;
	}
	if ((max > hardRollRateConstraint_) || (max < 0.0))
	{
		APLOG_WARN << "Roll rate constraint max violates hard constraint.";
		return false;
	}
	rollRateTargetConstraint_->setContraintValue(min / 180. * M_PI, max / 180. * M_PI);
	return true;
}

bool
RateCascade::tunePitchRateBounds(double min, double max)
{
	if ((min < -hardPitchRateConstraint_) || (min > 0.0))
	{
		APLOG_WARN << "Pitch rate constraint min violates hard constraint.";
		return false;
	}
	if ((max > hardPitchRateConstraint_) || (max < 0.0))
	{
		APLOG_WARN << "Pitch rate constraint max violates hard constraint.";
		return false;
	}
	pitchRateTargetConstraint_->setContraintValue(min / 180. * M_PI, max / 180. * M_PI);
	return true;
}

PIDStati
RateCascade::getPIDStatus()
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
	double yaw = sensorData_->attitude.z();
	double roll = sensorData_->attitude.x();
	double pitch = -sensorData_->attitude.y();

	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd(-roll, Vector3::UnitX()) * Eigen::AngleAxisd(-pitch, Vector3::UnitY())
			* Eigen::AngleAxisd(-yaw, Vector3::UnitZ());

	velocityBody = m * sensorData_->velocity;

	double bigV = velocityBody.norm();
	double smallV = velocityBody[1];

	beta_ = -asin(smallV / bigV);

	rollTarget_ = -atan2(bigV * controllerTarget_->yawRate, 9.81);

	controlEnv_.evaluate();
}

void
RateCascade::setManeuverOverride(const Override& override)
{
	for (auto& it : pids_)
		it.second->disableOverride();

	for (auto& it : outputs_)
		it.second->disableOverride();

	for (const auto& it : override.pid)
	{
		if (auto pid = findInMap(pids_, it.first))
			pid->second->overrideTarget(it.second);
	}

	for (const auto& it : override.output)
	{
		if (auto out = findInMap(outputs_, it.first))
			out->second->overrideOutput(it.second);
	}
}
