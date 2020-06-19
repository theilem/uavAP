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
#include <cpsCore/Utilities/Scheduler/IScheduler.h>

#include <uavAP/Core/DataHandling/Content.hpp>
#include "uavAP/FlightControl/SensingActuationIO/ISensingActuationIO.h"
#include "uavAP/FlightControl/Controller/PIDController/ManeuverPIDController/detail/ManeuverCascade.h"
#include "uavAP/FlightControl/Controller/PIDController/ManeuverPIDController/ManeuverPIDController.h"
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <cpsCore/Utilities/IPC/IPC.h>
#include "uavAP/Core/DataHandling/DataHandling.h"

std::shared_ptr<ManeuverPIDController>
ManeuverPIDController::create(const Configuration& config)
{
	auto flightController = std::make_shared<ManeuverPIDController>();
	flightController->configure(config);

	return flightController;
}

bool
ManeuverPIDController::configure(const Configuration& config)
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
		if (!checkIsSet<ISensingActuationIO, IScheduler, IPC>())
		{
			CPSLOG_ERROR << "PIDController: Missing Dependencies";
			return true;
		}
		if (!isSet<DataHandling>())
		{
			CPSLOG_DEBUG << "ManeuverPIDController: DataHandling not set. Debugging disabled.";
		}

		auto ipc = get<IPC>();

		controllerOutputPublisher_ = ipc->publishPackets("controller_output");

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = get<IPC>();
		overrideSubscription_ = ipc->subscribeOnPackets("override",
				std::bind(&ManeuverPIDController::onOverridePacket, this, std::placeholders::_1));

		if (!overrideSubscription_.connected())
		{
			CPSLOG_DEBUG << "Override not present.";
		}

		auto scheduler = get<IScheduler>();
//		scheduler->schedule(std::bind(&ManeuverPIDController::calculateControl, this),
//				Milliseconds(0), Milliseconds(10));



		if (auto dh = get<DataHandling>())
		{
			dh->addStatusFunction<std::map<PIDs, PIDStatus>>(
					std::bind(&IPIDCascade::getPIDStatus, pidCascade_), Content::PID_STATUS);
			dh->subscribeOnData<PIDTuning>(Content::TUNE_PID,
					std::bind(&ManeuverPIDController::tunePID, this, std::placeholders::_1));
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
ManeuverPIDController::calculateControl()
{
	auto sensAct = get<ISensingActuationIO>();

	if (!sensAct)
	{
		CPSLOG_ERROR << "PIDController: Failed to Locate FlightControlData";
		return;
	}

	sensorData_ = sensAct->getSensorData();

	Matrix3 m;
	m = AngleAxis(-sensorData_.attitude.x(), Vector3::UnitX())
			* AngleAxis(-sensorData_.attitude.y(), Vector3::UnitY())
			* AngleAxis(-sensorData_.attitude.z(), Vector3::UnitZ());

	accelerationInertial_ = m * sensorData_.acceleration;
	Lock targetLock(controllerTargetMutex_);

	if (!pidCascade_)
	{
		CPSLOG_ERROR << "ControlEnv not set.";
		return;
	}
	pidCascade_->evaluate();

//	controllerOutput_.sequenceNr = std::min(sensorData_.sequenceNr, controllerTarget_.sequenceNr);

	targetLock.unlock();

	sensAct->setControllerOutput(controllerOutput_);

	auto dp = get<DataPresentation>();
	controllerOutputPublisher_.publish(dp->serialize(controllerOutput_));
}

void
ManeuverPIDController::onOverridePacket(const Packet& packet)
{
	auto dp = get<DataPresentation>();
	auto override = dp->deserialize<Override>(packet);
	LockGuard lock(controllerTargetMutex_);
	pidCascade_->setManeuverOverride(override);
}

void
ManeuverPIDController::tunePID(const PIDTuning& params)
{
	pidCascade_->tunePID(static_cast<PIDs>(params.pid), params.params);
}
