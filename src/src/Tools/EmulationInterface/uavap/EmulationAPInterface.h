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
#include <boost/property_tree/ptree.hpp>
#include <uavAP/Core/IPC/Publisher.h>
#include <uavAP/Core/IPC/Subscription.h>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Runner/IRunnableObject.h>

#include <uavAP/API/ap_ext/ap_ext.h>
#include <uavAP/API/ap_ext/ServoMapping.h>
#include <uavAP/API/ChannelMixing.h>
#include <uavAP/Core/DataPresentation/DataPresentation.h>
#include <uavAP/Core/IDC/Serial/SerialIDC.h>
#include <uavAP/Core/SensorData.h>
#include <uavAP/Core/Scheduler/IScheduler.h>

class EmulationAPInterface: public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "emulation_interface";

	EmulationAPInterface();

	~EmulationAPInterface();

	static std::shared_ptr<EmulationAPInterface>
	create(const Configuration& config);

	bool
	configure(const Configuration& config);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

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
	ObjectHandle<IDataPresentation<Content, Target>> dataPresentation_;
	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<INetworkLayer> idc_;

	std::string serialPort_;
	Sender actuationSender_;

	const int numChannels_;
	uint32_t lastSequenceNr_;
};

#endif /* AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_H_ */
