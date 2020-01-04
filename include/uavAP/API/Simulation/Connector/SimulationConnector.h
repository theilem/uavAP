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
 * SimulationConnector.h
 *
 *  Created on: Jul 28, 2017
 *      Author: mircot
 */

#ifndef UAVAP_SIMULATION_SIMULATIONCONNECTOR_H_
#define UAVAP_SIMULATION_SIMULATIONCONNECTOR_H_
#include <boost/smart_ptr/shared_ptr.hpp>
#include <uavAP/Core/IDC/IDCSender.h>
#include <uavAP/Core/IDC/INetworkLayer.h>
#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/IPC/Subscription.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Scheduler/IScheduler.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include <iostream>
#include <fstream>

#include <memory>

class IPC;
class IScheduler;
class ITimeProvider;
class SerialNetworkLayer;
class ChannelMixing;
class SensorData;

class SimulationConnector: public IRunnableObject, public IAggregatableObject
{
public:

	static constexpr TypeId typeId = "connector";

	SimulationConnector();

	~SimulationConnector();

	static std::shared_ptr<SimulationConnector>
	create(const Configuration& config);

	bool
	configure(const Configuration& config);

	bool
	run(RunStage stage) override;

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

private:

	void
	actuate(const ControllerOutput& out);

	void
	sense(const Packet& packet);

	void
	receivePackets(const Packet& out);

	void
	actuationDisconnect();

	void
	tryConnectActuation();

	void
	tryConnectCommunication();

	void
	startLogging();

	void
	stopLogging();

	void
	logData(const SensorData& data);

	ObjectHandle<IPC> ipc_;
	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<ITimeProvider> timeProvider_;
	ObjectHandle<SerialNetworkLayer> idc_;
	ObjectHandle<ChannelMixing> channelMixing_;
	ObjectHandle<DataPresentation> dataPresentation_;

	Subscription actuationSubscription_;
	Subscription communicationSubscription_;
	Publisher sensorPublisher_;

	Sender actuationSender_;

	Event actuationResetEvent_;

	std::string serialPort_;

	bool logging_;
	std::string loggingPath_;
	std::ofstream fileStream_;
};

#endif /* UAVAP_SIMULATION_SIMULATIONCONNECTOR_H_ */
