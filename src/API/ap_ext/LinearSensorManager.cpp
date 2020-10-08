//
// Created by mirco on 28.08.20.
//

#include "uavAP/API/ap_ext/LinearSensorManager.h"

bool
LinearSensorManager::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			for (const auto& [sensorName, sensorParams]: params.sensors())
			{
				LinearSensor sensor;
				sensor.setParams(sensorParams);
				sensor.initialize();
				sensors_.emplace_back(std::make_pair(sensorName, sensor));
			}
			break;
		}
		default:
			break;
	}

	return false;
}