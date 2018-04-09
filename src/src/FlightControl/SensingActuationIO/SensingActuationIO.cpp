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
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/SensingActuationIO/SensingActuationIO.h"

SensingActuationIO::SensingActuationIO()
{
}

std::shared_ptr<SensingActuationIO>
SensingActuationIO::create(const boost::property_tree::ptree&)
{
	return std::make_shared<SensingActuationIO>();
}

void
SensingActuationIO::notifyAggregationOnUpdate(Aggregator& agg)
{
	ipc_.setFromAggregationIfNotSet(agg);
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
		auto ipc = ipc_.get();

		actuationPublisher_ = ipc->publishOnSharedMemory<ControllerOutput>("actuation");

		break;
	}
	case RunStage::NORMAL:
	{
		auto ipc = ipc_.get();
		sensorSubscription_ = ipc->subscribeOnSharedMemory<SensorData>("sensor_data", boost::bind(&SensingActuationIO::onSensorData, this, _1));
		if (!sensorSubscription_.connected())
		{
			APLOG_ERROR << "SensorData in shared memory missing. Cannot continue.";
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

void
SensingActuationIO::setControllerOutput(const ControllerOutput& out)
{
	actuationPublisher_.publish(out);
}

void
SensingActuationIO::onSensorData(const SensorData& data)
{
	boost::unique_lock<boost::shared_mutex> lock(mutex);
	sensorData = data;
	lock.unlock();
	onSensorData_(data);
}

boost::signals2::connection
SensingActuationIO::subscribeOnSensorData(const OnSensorData::slot_type& slot)
{
	return onSensorData_.connect(slot);
}
