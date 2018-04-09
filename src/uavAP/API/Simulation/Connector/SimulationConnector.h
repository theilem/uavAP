/*
 * SimulationConnector.h
 *
 *  Created on: Jul 28, 2017
 *      Author: mircot
 */

#ifndef UAVAP_SIMULATION_SIMULATIONCONNECTOR_H_
#define UAVAP_SIMULATION_SIMULATIONCONNECTOR_H_
#include <boost/smart_ptr/shared_ptr.hpp>
#include "uavAP/Core/DataPresentation/Content.h"
#include "uavAP/Core/IDC/IInterDeviceComm.h"
#include "uavAP/Core/IDC/Sender.h"
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
class SerialIDC;
class ChannelMixing;
class SensorData;

class SimulationConnector: public IRunnableObject, public IAggregatableObject
{
public:

	SimulationConnector();

	~SimulationConnector();

	static std::shared_ptr<SimulationConnector>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	bool
	run(RunStage stage) override;

	void
	notifyAggregationOnUpdate(Aggregator& agg) override;

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
	ObjectHandle<SerialIDC> idc_;
	ObjectHandle<ChannelMixing> channelMixing_;
	ObjectHandle<IDataPresentation<Content,Target>> dataPresentation_;

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
