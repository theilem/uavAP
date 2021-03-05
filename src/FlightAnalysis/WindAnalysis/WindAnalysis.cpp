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
 * WindAnalysis.cpp
 *
 *  Created on: Oct 19, 2019
 *      Author: simonyu
 */

#include "uavAP/Core/SensorData.h"
#include "uavAP/MissionControl/WindAnalysis/WindAnalysis.h"
#include <cpsCore/Utilities/LockTypes.hpp>
#include <cpsCore/Utilities/IPC/IPC.h>

bool
WindAnalysis::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!isSet<IPC>())
		{
			CPSLOG_ERROR << "WindAnalysis: IPC missing.";
			return true;
		}

		auto ipc = get<IPC>();

		windInfoPublisher_ = ipc->publish<WindInfo>("wind_info");

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = get<IPC>();

		sensorDataSubscription_ = ipc->subscribe<SensorData>("sensor_data",
				std::bind(&WindAnalysis::onSensorData, this, std::placeholders::_1));

		if (!sensorDataSubscription_.connected())
		{
			CPSLOG_ERROR << "WindAnalysis: Sensor Data Subscription Missing.";
			return true;
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

WindInfo
WindAnalysis::getWindInfo() const
{
	LockGuard lg(windInfoMutex_);
	return windInfo_;
}

WindAnalysisStatus
WindAnalysis::getWindAnalysisStatus() const
{
	LockGuard lg(windAnalysisStatusMutex_);
	return windAnalysisStatus_;
}

void
WindAnalysis::setWindAnalysisStatus(const WindAnalysisStatus& windAnalysisStatus)
{
	if (windAnalysisStatus.manual)
	{
		if (std::isnan(windAnalysisStatus.velocity.x())
				|| std::isnan(windAnalysisStatus.velocity.y())
				|| std::isnan(windAnalysisStatus.velocity.z()))
		{
			if (!std::isnan(windAnalysisStatus.speed) && !std::isnan(windAnalysisStatus.direction))
			{
				Lock windInfoLock(windInfoMutex_);
				windInfo_.velocity.x() = windAnalysisStatus.speed
						* cos(windAnalysisStatus.direction);
				windInfo_.velocity.y() = windAnalysisStatus.speed
						* sin(windAnalysisStatus.direction);
				windInfo_.velocity.z() = 0;

				Lock windAnalysisStatusLock(windAnalysisStatusMutex_);
				windAnalysisStatus_ = windAnalysisStatus;
				windAnalysisStatus_.velocity = windInfo_.velocity;
				windAnalysisStatusLock.unlock();
				windInfoLock.unlock();
			}
		}
		else
		{
			Lock windInfoLock(windInfoMutex_);
			windInfo_.velocity = windAnalysisStatus.velocity;

			Lock windAnalysisStatusLock(windAnalysisStatusMutex_);
			windAnalysisStatus_ = windAnalysisStatus;
			windAnalysisStatus_.speed = windAnalysisStatus.velocity.norm();
			windAnalysisStatus_.direction = atan2(windAnalysisStatus.velocity.y(),
					windAnalysisStatus.velocity.x());
			windAnalysisStatusLock.unlock();
			windInfoLock.unlock();
		}
	}
	else
	{
		if (!std::isnan(windAnalysisStatus.direction))
		{
			Lock windAnalysisStatusLock(windAnalysisStatusMutex_);
			if (windAnalysisStatus_.manual)
			{
				windAnalysisStatus_.reset();
			}
			windAnalysisStatusLock.unlock();
		}
	}
}

void
WindAnalysis::onSensorData(const SensorData& sensorData)
{
	Lock windAnalysisStatusLock(windAnalysisStatusMutex_);
	WindAnalysisStatus windAnalysisStatus = windAnalysisStatus_;
	windAnalysisStatusLock.unlock();

	Lock windInfoLock(windInfoMutex_);
	WindInfo windInfo = windInfo_;
	windInfoLock.unlock();

	if (!windAnalysisStatus.manual)
	{
		Vector3 airSpeedAttitude;
		Vector3 airSpeedInertial;

		airSpeedAttitude = sensorData.attitude;

		if (params.useAlpha())
		{
			airSpeedAttitude.y() -= sensorData.angleOfAttack;
		}

		if (params.useBeta())
		{
			airSpeedAttitude.z() -= sensorData.angleOfSideslip;
		}

		airSpeedInertial.x() = sensorData.airSpeed * cos(airSpeedAttitude.y())
				* cos(airSpeedAttitude.z());
		airSpeedInertial.y() = sensorData.airSpeed * cos(airSpeedAttitude.y())
				* sin(airSpeedAttitude.z());
		airSpeedInertial.z() = sensorData.airSpeed * sin(airSpeedAttitude.y());

		auto diff = sensorData.timestamp - timeStamp_;
		timeStamp_ = sensorData.timestamp;
		windInfo.velocity = windFilter_.update(sensorData.velocity - airSpeedInertial, diff);

		Lock windAnalysisStatusLock(windAnalysisStatusMutex_);
		windAnalysisStatus_.velocity = windInfo.velocity;
		windAnalysisStatus_.speed = windInfo.velocity.norm();
		windAnalysisStatus_.direction = atan2(windInfo.velocity.y(), windInfo.velocity.x());
		windAnalysisStatusLock.unlock();

		Lock windInfoLock(windInfoMutex_);
		windInfo_ = windInfo;
		windInfoLock.unlock();
	}

	windInfoPublisher_.publish(windInfo);
}