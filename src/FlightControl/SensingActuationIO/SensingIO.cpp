//
// Created by mirco on 26.02.21.
//

#include "uavAP/FlightControl/SensingActuationIO/SensingIO.h"
#include "uavAP/Core/DataHandling/Content.hpp"
#include <cpsCore/Utilities/IPC/IPC.h>

bool
SensingIO::run(RunStage stage)
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
			auto ipc = get<IPC>();

			break;
		}
		case RunStage::NORMAL:
		{
			auto ipc = get<IPC>();
			sensorSubscription_ = ipc->subscribe<SensorData>("sensor_data", [this](const auto& sd)
			{ onSensorData(sd); });
			powerSubscription_ = ipc->subscribe<PowerData>("power_data", [this](const auto& pd)
			{ powerData_ = pd; });
			servoSubscription_ = ipc->subscribe<ServoData>("servo_data", [this](const auto& sd)
			{ servoData_ = sd; });
			if (!sensorSubscription_.connected())
			{
				CPSLOG_ERROR << "SensorData in shared memory missing. Cannot continue.";
				return true;
			}
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
SensingIO::onSensorData(const SensorData& data)
{
	std::unique_lock<SharedMutex> lock(mutex_);
	sensorData_ = data;
	lock.unlock();
	onSensorData_(data);
}

boost::signals2::connection
SensingIO::subscribeOnSensorData(const OnSensorData::slot_type& slot)
{
	return onSensorData_.connect(slot);
}

SensorData
SensingIO::getSensorData() const
{
	std::unique_lock<SharedMutex> lock(mutex_);
	return sensorData_;
}