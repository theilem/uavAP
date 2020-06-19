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
 * SensingActuationIO.cpp
 *
 *  Created on: Jul 26, 2017
 *      Author: mircot
 */
#include <uavAP/FlightControl/Controller/AdvancedControl.h>
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/SensingActuationIO/SensingActuationIO.h"
#include <uavAP/Core/DataHandling/DataHandling.h>
#include <cpsCore/Utilities/IPC/IPC.h>
#include "uavAP/Core/DataHandling/Content.hpp"

bool
SensingActuationIO::run(RunStage stage)
{

	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!isSet<IPC>())
		{
			CPSLOG_ERROR << "SensingActuationIO: IPC is missing.";
			return true;
		}
		if (!isSet<DataHandling>())
		{
			CPSLOG_DEBUG << "SensingActuationIO: DataHandling not set. Debugging disabled.";
		}
		auto ipc = get<IPC>();

		actuationPublisher_ = ipc->publish<ControllerOutput>("actuation");
		advancedControlPublisher_ = ipc->publish<AdvancedControl>("advanced_control");

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = get<IPC>();
		sensorSubscription_ = ipc->subscribe<SensorData>("sensor_data",
				std::bind(&SensingActuationIO::onSensorData, this, std::placeholders::_1));
		if (!sensorSubscription_.connected())
		{
			CPSLOG_ERROR << "SensorData in shared memory missing. Cannot continue.";
			return true;
		}
		if (auto dh = get<DataHandling>())
		{
			dh->addStatusFunction<SensorData>(
					std::bind(&SensingActuationIO::getSensorData, this), Content::SENSOR_DATA);
			dh->subscribeOnData<AdvancedControl>(Content::ADVANCED_CONTROL,
					std::bind(&SensingActuationIO::onAdvancedControl, this, std::placeholders::_1));
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
SensingActuationIO::setControllerOutput(const ControllerOutput& out)
{
	actuationPublisher_.publish(out);
}

void
SensingActuationIO::onSensorData(const SensorData& data)
{
	std::unique_lock<SharedMutex> lock(mutex_);
	sensorData_ = data;
	lock.unlock();
	onSensorData_(data);
}

boost::signals2::connection
SensingActuationIO::subscribeOnSensorData(const OnSensorData::slot_type& slot)
{
	return onSensorData_.connect(slot);
}

SensorData
SensingActuationIO::getSensorData() const
{
	std::unique_lock<SharedMutex> lock(mutex_);
	return sensorData_;
}

void
SensingActuationIO::onAdvancedControl(const AdvancedControl& control)
{
	CPSLOG_TRACE << "Camber Control: " << EnumMap<CamberControl>::convert(control.camberSelection);
	CPSLOG_TRACE << "Special Control: " << EnumMap<SpecialControl>::convert(control.specialSelection);
	CPSLOG_TRACE << "Throw Control: " << EnumMap<ThrowsControl>::convert(control.throwsSelection);
	CPSLOG_TRACE << "Camber Value: " << control.camberValue;
	CPSLOG_TRACE << "Special Value: " << control.specialValue;
	advancedControlPublisher_.publish(control);
}
