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
 * ManeuverPIDController.cpp
 *
 *  Created on: Sep 15, 2017
 *      Author: mircot
 */
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingActuationIO.h"
#include "uavAP/FlightControl/Controller/PIDController/ManeuverPIDController/detail/ManeuverCascade.h"
#include "uavAP/FlightControl/Controller/PIDController/ManeuverPIDController/ManeuverPIDController.h"
#include "uavAP/Core/LockTypes.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/DataPresentation/BinarySerialization.hpp"

ManeuverPIDController::ManeuverPIDController()
{
}

std::shared_ptr<ManeuverPIDController>
ManeuverPIDController::create(const boost::property_tree::ptree& config)
{
	auto flightController = std::make_shared<ManeuverPIDController>();
	flightController->configure(config);

	return flightController;
}

bool
ManeuverPIDController::configure(const boost::property_tree::ptree& config)
{
	pidCascade_ = std::make_shared<ManeuverCascade>(&sensorData_, velocityInertial_,
			accelerationInertial_, &controllerTarget_, &controllerOutput_);

	return pidCascade_->configure(config);
}

bool
ManeuverPIDController::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!sensAct_.isSet())
		{
			APLOG_ERROR << "PIDController: Failed to Load SensingActuationIO";
			return true;
		}
		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "PIDController: Failed to Load Scheduler";
			return true;
		}
		if (!ipc_.isSet())
		{
			APLOG_ERROR << "PIDController: ipc missing";
			return true;
		}

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = ipc_.get();
		overrideSubscription_ = ipc->subscribeOnPacket("override",
				std::bind(&ManeuverPIDController::onOverridePacket, this, std::placeholders::_1));

		if (!overrideSubscription_.connected())
		{
			APLOG_DEBUG << "Override not present.";
		}

		auto scheduler = scheduler_.get();
//		scheduler->schedule(std::bind(&ManeuverPIDController::calculateControl, this),
//				Milliseconds(0), Milliseconds(10));

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
ManeuverPIDController::setControllerTarget(const ControllerTarget& target)
{
//	LockGuard lock(controllerTargetMutex_);
	controllerTarget_ = target;
	calculateControl();
}

ControllerOutput
ManeuverPIDController::getControllerOutput()
{
	return controllerOutput_;
}

void
ManeuverPIDController::notifyAggregationOnUpdate(const Aggregator& agg)
{
	sensAct_.setFromAggregationIfNotSet(agg);
	scheduler_.setFromAggregationIfNotSet(agg);
	ipc_.setFromAggregationIfNotSet(agg);
}

std::shared_ptr<IPIDCascade>
ManeuverPIDController::getCascade()
{
	return pidCascade_;
}

void
ManeuverPIDController::calculateControl()
{
	auto sensAct = sensAct_.get();

	if (!sensAct)
	{
		APLOG_ERROR << "PIDController: Failed to Locate FlightControlData";
		return;
	}

	sensorData_ = sensAct->getSensorData();

	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd(-sensorData_.attitude.x(), Vector3::UnitX())
			* Eigen::AngleAxisd(-sensorData_.attitude.y(), Vector3::UnitY())
			* Eigen::AngleAxisd(-sensorData_.attitude.z(), Vector3::UnitZ());

	accelerationInertial_ = m * sensorData_.acceleration;
	Lock targetLock(controllerTargetMutex_);

	if (!pidCascade_)
	{
		APLOG_ERROR << "ControlEnv not set.";
		return;
	}
	pidCascade_->evaluate();

	controllerOutput_.sequenceNr = std::min(sensorData_.sequenceNr, controllerTarget_.sequenceNr);

	targetLock.unlock();

	sensAct->setControllerOutput(controllerOutput_);
}

void
ManeuverPIDController::onOverridePacket(const Packet& packet)
{
	auto override = dp::deserialize<Override>(packet);
	LockGuard lock(controllerTargetMutex_);
	pidCascade_->setManeuverOverride(override);
}
