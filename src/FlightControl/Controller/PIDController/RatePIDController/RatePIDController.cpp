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
 * RatePIDController.cpp
 *
 *  Created on: Sep 15, 2017
 *      Author: mircot
 */

#include "uavAP/FlightControl/SensingActuationIO/ISensingActuationIO.h"
#include "uavAP/FlightControl/Controller/PIDController/RatePIDController/detail/RateCascade.h"
#include "uavAP/FlightControl/Controller/PIDController/RatePIDController/RatePIDController.h"
#include "uavAP/Core/DataHandling/DataHandling.h"
#include <cpsCore/Utilities/IPC/IPC.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <uavAP/Core/DataHandling/Content.hpp>

RatePIDController::RatePIDController()
{
}

std::shared_ptr<RatePIDController>
RatePIDController::create(const Configuration& config)
{
	auto flightController = std::make_shared<RatePIDController>();
	flightController->configure(config);

	return flightController;
}

bool
RatePIDController::configure(const Configuration& config)
{
	pidCascade_ = std::make_shared<RateCascade>(&sensorData_, velocityInertial_,
			accelerationInertial_, &controllerTarget_, &controllerOutput_);

	return pidCascade_->configure(config);
}

bool
RatePIDController::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!checkIsSet<ISensingActuationIO, IScheduler, IPC>())
		{
			CPSLOG_ERROR << "RatePIDController: missing dependencies";
			return true;
		}
		if (!isSet<DataHandling>())
		{
			CPSLOG_DEBUG << "RatePIDController: DataHandling not set. Debugging disabled.";
		}

		auto ipc = get<IPC>();

		controllerOutputPublisher_ = ipc->publish<ControllerOutput>("controller_output");
		pidStatiPublisher_ = ipc->publishPackets("pid_stati");

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = get<IPC>();
		overrideSubscription_ = ipc->subscribeOnPackets("override",
				std::bind(&RatePIDController::onOverridePacket, this, std::placeholders::_1));

		if (!overrideSubscription_.connected())
		{
			CPSLOG_WARN << "RatePIDController: maneuver activation subscription failed";
//			return true;
		}

		if (auto dh = get<DataHandling>())
		{
			dh->addStatusFunction<std::map<PIDs, PIDStatus>>(
					std::bind(&IPIDCascade::getPIDStatus, pidCascade_), Content::PID_STATUS);
			dh->subscribeOnData<PIDTuning>(Content::TUNE_PID,
					std::bind(&RatePIDController::tunePID, this, std::placeholders::_1));
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
RatePIDController::setControllerTarget(const ControllerTarget& target)
{
	controllerTarget_ = target;
	calculateControl();
}

ControllerOutput
RatePIDController::getControllerOutput()
{
	return controllerOutput_;
}

std::shared_ptr<IPIDCascade>
RatePIDController::getCascade()
{
	return pidCascade_;
}

void
RatePIDController::calculateControl()
{
	auto sensAct = get<ISensingActuationIO>();

	if (!sensAct)
	{
		CPSLOG_ERROR << "RatePIDController: Failed to Locate FlightControlData";
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
		CPSLOG_ERROR << "ControlEnv not set.";
		return;
	}
	pidCascade_->evaluate();

//	controllerOutput_.sequenceNr = std::min(sensorData_.sequenceNr, controllerTarget_.sequenceNr);

	targetLock.unlock();

	sensAct->setControllerOutput(controllerOutput_);
	controllerOutputPublisher_.publish(controllerOutput_);

	if (auto dp = get<DataPresentation>())
	{
		pidStatiPublisher_.publish(dp->serialize(pidCascade_->getPIDStatus()));
	}
	else
	{
		CPSLOG_ERROR << "RatePIDController: Data Presentation Missing.";
	}
}

void
RatePIDController::onOverridePacket(const Packet& packet)
{
	if (auto dp = get<DataPresentation>())
	{
		auto override = dp->deserialize<Override>(packet);
		LockGuard lock(controllerTargetMutex_);
		pidCascade_->setManeuverOverride(override);
	}
	else
	{
		CPSLOG_ERROR << "RatePIDController: Data Presentation Missing.";
	}
}

void
RatePIDController::tunePID(const PIDTuning& params)
{
	pidCascade_->tunePID(static_cast<PIDs>(params.pid), params.params);
}
