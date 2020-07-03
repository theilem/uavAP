//
// Created by mirco on 03.07.20.
//

#include "EmulationDirectInterface.h"
#include <cpsCore/Utilities/IDC/IDC.h>

#include "uavAP/API/AggregatableAutopilotAPI.h"

#include <uavAP/Core/SensorData.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>
#include <cpsCore/Utilities/DataPresentation/BinarySerialization.hpp>

bool
EmulationDirectInterface::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			if (!checkIsSet<AggregatableAutopilotAPI, IDC>())
			{
				CPSLOG_ERROR << "Missing dependencies";
				return true;
			}
			auto idc = get<IDC>();

			controlSender_ = idc->createSender(params.controllerOutputTarget());
			idc->subscribeOnPacket(params.sensorDataTarget(), [this](const auto& p){this->onSensorDataPacket(p);});
			break;
		}
		case RunStage::NORMAL:
		{
			auto api = get<AggregatableAutopilotAPI>();
			api->subscribeOnControllerOut([this](const ControllerOutput& co)
										  { controlSender_.sendPacket(dp::serialize<ControllerOutput>(co)); });


			break;
		}
		default:
			break;
	}
	return false;
}

void
EmulationDirectInterface::onSensorDataPacket(const Packet& packet)
{
	auto api = get<AggregatableAutopilotAPI>();
	auto sd = dp::deserialize<SensorData>(packet);

	api->setSensorData(sd);
}
