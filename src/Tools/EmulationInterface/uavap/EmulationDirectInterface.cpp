//
// Created by mirco on 03.07.20.
//

#include "EmulationDirectInterface.h"
#include <cpsCore/Utilities/IDC/IDC.h>

#include "uavAP/API/AggregatableAutopilotAPI.h"

#include <uavAP/Core/SensorData.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <uavAP/Core/DataHandling/Content.hpp>

bool
EmulationDirectInterface::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			if (!checkIsSetAll())
			{
				CPSLOG_ERROR << "Missing dependencies";
				return true;
			}
			auto idc = get<IDC>();

			controlSender_ = idc->createSender(params.controllerOutputTarget());
			idc->subscribeOnPacket(params.sensorDataTarget(), [this](const auto& p)
			{ this->onSensorDataPacket(p); });
			break;
		}
		case RunStage::NORMAL:
		{
			auto api = get<AggregatableAutopilotAPI>();
			api->subscribeOnControllerOut([this](const ControllerOutput& co)
										  {sendControllerOutput(co); });


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
	auto dp = get<DataPresentation>();

	auto p = packet;
	auto content = dp->extractHeader<Content>(p);

	switch (content)
	{
		case Content::SENSOR_DATA:
			api->setSensorData(dp->deserialize<SensorData>(p));
			break;
		case Content::SERVO_DATA:
			api->setServoData(dp->deserialize<ServoData>(p));
			break;
		case Content::POWER_DATA:
			api->setPowerData(dp->deserialize<PowerData>(p));
			break;
		default:
			CPSLOG_WARN << "Unknown packet of content: " << static_cast<int>(content);
	}
}

void
EmulationDirectInterface::sendControllerOutput(const ControllerOutput& output)
{
	auto dp = get<DataPresentation>();

	auto packet = dp->serialize(output);
	dp->addHeader(packet, Content::CONTROLLER_OUTPUT);
	controlSender_.sendPacket(packet);

}
