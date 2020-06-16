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

class INetworkLayer;

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
