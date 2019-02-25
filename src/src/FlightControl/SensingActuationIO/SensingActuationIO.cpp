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
#include <uavAP/Core/DataHandling/DataHandling.h>
#include <uavAP/Core/LockTypes.h>
#include <uavAP/FlightControl/Controller/AdvancedControl.h>
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/SensingActuationIO/SensingActuationIO.h"

SensingActuationIO::SensingActuationIO()
{
}

void
SensingActuationIO::notifyAggregationOnUpdate(const Aggregator& agg)
{
	ipc_.setFromAggregationIfNotSet(agg);
	dataHandling_.setFromAggregationIfNotSet(agg);
}

bool
SensingActuationIO::run(RunStage stage)
{

	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!ipc_.isSet())
		{
			APLOG_ERROR << "SensingActuationIO: IPC is missing.";
			return true;
		}
		if (!dataHandling_.isSet())
		{
			APLOG_DEBUG << "SensingActuationIO: DataHandling not set. Debugging disabled.";
		}
		auto ipc = ipc_.get();

		actuationPublisher_ = ipc->publishOnSharedMemory<ControllerOutput>("actuation");
		advancedControlPublisher_ = ipc->publishOnSharedMemory<AdvancedControl>("advanced_control");

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = ipc_.get();
		sensorSubscription_ = ipc->subscribeOnSharedMemory<SensorData>("sensor_data",
				boost::bind(&SensingActuationIO::onSensorData, this, _1));
		if (!sensorSubscription_.connected())
		{
			APLOG_ERROR << "SensorData in shared memory missing. Cannot continue.";
			return true;
		}
		if (auto dh = dataHandling_.get())
		{
			dh->addStatusFunction<SensorData>(
					std::bind(&SensingActuationIO::getSensorData, this));
			dh->subscribeOnCommand<AdvancedControl>(Content::ADVANCED_CONTROL,
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
	boost::unique_lock<boost::shared_mutex> lock(mutex_);
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
	SharedLockGuard lock(mutex_);
	return sensorData_;
}

void
SensingActuationIO::onAdvancedControl(const AdvancedControl& control)
{
	APLOG_TRACE << "Camber Control: " << EnumMap<CamberControl>::convert(control.camberSelection);
	APLOG_TRACE << "Special Control: " << EnumMap<SpecialControl>::convert(control.specialSelection);
	APLOG_TRACE << "Throw Control: " << EnumMap<ThrowsControl>::convert(control.throwsSelection);
	APLOG_TRACE << "Camber Value: " << control.camberValue;
	APLOG_TRACE << "Special Value: " << control.specialValue;
	advancedControlPublisher_.publish(control);
}
