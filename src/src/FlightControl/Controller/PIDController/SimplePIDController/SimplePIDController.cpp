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
/**
 *  @file		SimpleSimplePIDController.cpp
 *  @author  	Mirco Theile
 *  @date      	05 June 2017
 *  @brief      UAV Autopilot PID Controller Source File
 *
 *  Description
 */

#include <uavAP/Core/DataHandling/DataHandling.h>
#include <iostream>
#include <cmath>
#include <mutex>

#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/LockTypes.h"
#include "uavAP/Core/Scheduler/IScheduler.h"
#include "uavAP/FlightControl/Controller/PIDController/SimplePIDController/detail/AirplaneSimplePIDCascade.h"
//#include "uavAP/FlightControl/Controller/PIDController/SimplePIDController/detail/HelicopterSimplePIDCascade.h"
#include "uavAP/FlightControl/Controller/PIDController/SimplePIDController/SimplePIDController.h"

SimplePIDController::SimplePIDController() :
		airplane_(true)
{
}

std::shared_ptr<SimplePIDController>
SimplePIDController::create(const boost::property_tree::ptree& configuration)
{
	auto flightController = std::make_shared<SimplePIDController>();
	flightController->configure(configuration);

	return flightController;
}

bool
SimplePIDController::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper propertyMapper(config);
	propertyMapper.add<bool>("airplane", airplane_, false);

	if (airplane_)
	{
		pidCascade_ = std::make_shared<AirplaneSimplePIDCascade>(&sensorData_, velocityInertial_,
				accelerationInertial_, &controllerTarget_, &controllerOutput_);
	}
	else
	{
//		pidCascade_ = std::make_shared<HelicopterSimplePIDCascade>(&sensorData_, &controllerTarget_,
//				&controllerOutput_);
	}

	return pidCascade_->configure(config);
}

bool
SimplePIDController::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!sensAct_.isSet())
		{
			APLOG_ERROR << "SimplePIDController: Failed to Load SensingActuationIO";

			return true;
		}
		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "SimplePIDController: Failed to Load Scheduler";

			return true;
		}
		if (!dataHandling_.isSet())
		{
			APLOG_DEBUG << "SimplePIDController: DataHandling not set. Debugging disabled.";
		}

		break;
	}
	case RunStage::NORMAL:
	{
		auto scheduler = scheduler_.get();
		scheduler->schedule(std::bind(&SimplePIDController::calculateControl, this),
				Milliseconds(0), Milliseconds(10));

		if (auto dh = dataHandling_.get())
		{
			dh->addStatusFunction<std::map<PIDs, PIDStatus>>(
					std::bind(&IPIDCascade::getPIDStatus, pidCascade_));
			dh->subscribeOnCommand<PIDTuning>(Content::TUNE_PID,
					std::bind(&SimplePIDController::tunePID, this, std::placeholders::_1));
		}

		break;
	}
	case RunStage::FINAL:
	{
		break;
	}
	default:
	{
		break;
	}
	}

	return false;
}

void
SimplePIDController::setControllerTarget(const ControllerTarget& target)
{
	LockGuard lock(controllerTargetMutex_);
	controllerTarget_ = target;
}

ControllerOutput
SimplePIDController::getControllerOutput()
{
	return controllerOutput_;
}

std::shared_ptr<IPIDCascade>
SimplePIDController::getCascade()
{
	return pidCascade_;
}

void
SimplePIDController::calculateControl()
{
	auto sensAct = sensAct_.get();

	if (!sensAct)
	{
		APLOG_ERROR << "SimplePIDController: Failed to Locate FlightControlData";

		return;
	}

	sensorData_ = sensAct->getSensorData();

	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd(sensorData_.attitude.x(), Vector3::UnitX())
			* Eigen::AngleAxisd(sensorData_.attitude.y(), Vector3::UnitY())
			* Eigen::AngleAxisd(sensorData_.attitude.z(), Vector3::UnitZ());

	velocityInertial_ = m.transpose() * sensorData_.velocity;
	accelerationInertial_ = m.transpose() * sensorData_.acceleration;
	accelerationInertial_[2] *= -1;
	Lock targetLock(controllerTargetMutex_);

	if (!pidCascade_)
	{
		APLOG_ERROR << "ControlEnv not set.";
		return;
	}
	pidCascade_->evaluate();
	targetLock.unlock();

	sensAct->setControllerOutput(controllerOutput_);
}

void
SimplePIDController::notifyAggregationOnUpdate(const Aggregator& agg)
{
	sensAct_.setFromAggregationIfNotSet(agg);
	scheduler_.setFromAggregationIfNotSet(agg);
	dataHandling_.setFromAggregationIfNotSet(agg);
}

void
SimplePIDController::tunePID(const PIDTuning& params)
{
	pidCascade_->tunePID(static_cast<PIDs>(params.pid), params.params);
}
