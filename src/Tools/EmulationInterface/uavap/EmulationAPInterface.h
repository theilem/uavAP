/*
 * AutopilotInterface.h
 *
 *  Created on: Nov 25, 2017
 *      Author: mircot
 */

#ifndef AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_H_
#define AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_H_

#include <uavAP/API/ap_ext/ap_ext.h>
#include <uavAP/API/ChannelMixing.h>
#include <uavAP/Core/SensorData.h>
#include <cpsCore/cps_object>
#include "EmulationAPInterfaceParams.h"

class DataPresentation;
class IScheduler;
class IDC;

class EmulationAPInterface:
				public ConfigurableObject<EmulationAPInterfaceParams>,
				public IRunnableObject,
				public AggregatableObject<IDC, DataPresentation, IScheduler>
{
public:

	static constexpr TypeId typeId = "emulation_interface";

	EmulationAPInterface();

	~EmulationAPInterface() override;

	bool
	run(RunStage stage) override;

private:

	void
	onPacket(const Packet& packet);

	void
	onSensorData(const SensorData& sensorData);

	void
	sendActuation();

	ControllerOutput
	reverseChannelMixing(unsigned long* pwm);

	bool setup_;

	data_sample_t dataSample_;

	ChannelMixing channelMixing_;
	ServoMapping servoMapping_;

	std::string serialPort_;
	Sender actuationSender_;

	const int numChannels_;
};

#endif /* AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_H_ */
