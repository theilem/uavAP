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

#include <iostream>
#include <cmath>

#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/Scheduler/IScheduler.h"
#include "uavAP/FlightControl/Controller/PIDController/SimplePIDController/detail/AirplaneSimplePIDCascade.h"
#include "uavAP/FlightControl/Controller/PIDController/SimplePIDController/SimplePIDController.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingActuationIO.h"
#include "uavAP/Core/Object/AggregatableObjectImpl.hpp"

SimplePIDController::SimplePIDController() :
		airplane_(true)
{
}

std::shared_ptr<SimplePIDController>
SimplePIDController::create(const Configuration& configuration)
{
	auto flightController = std::make_shared<SimplePIDController>();
	flightController->configure(configuration);

	return flightController;
}

bool
SimplePIDController::configure(const Configuration& config)
{
	PropertyMapper<Configuration> propertyMapper(config);

	pidCascade_ = std::make_shared<AirplaneSimplePIDCascade>(&sensorData_, velocityInertial_,
			accelerationInertial_, &controllerTarget_, &controllerOutput_);

	return pidCascade_->configure(config);
}

bool
SimplePIDController::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!isSet<ISensingActuationIO>())
		{
			APLOG_ERROR << "SimplePIDController: Failed to Load SensingActuationIO";

			return true;
		}
//		if (!isSet<IScheduler>())
//		{
//			APLOG_ERROR << "SimplePIDController: Failed to Load Scheduler";
//
//			return true;
//		}

		break;
	}
	case RunStage::NORMAL:
	{
//		auto scheduler = get<IScheduler>();
//		scheduler->schedule(std::bind(&SimplePIDController::calculateControl, this), Milliseconds(0),
//				Milliseconds(10));

		auto io = get<ISensingActuationIO>();
		io->subscribeOnSensorData(std::bind(&SimplePIDController::calculateControl, this));

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
	auto sensAct = get<ISensingActuationIO>();

	if (!sensAct)
	{
		APLOG_ERROR << "SimplePIDController: Failed to Locate FlightControlData";

		return;
	}

	sensorData_ = sensAct->getSensorData();

	Matrix3 m;
	m = AngleAxis(sensorData_.attitude.x(), Vector3::UnitX())
			* AngleAxis(sensorData_.attitude.y(), Vector3::UnitY())
			* AngleAxis(sensorData_.attitude.z(), Vector3::UnitZ());

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
