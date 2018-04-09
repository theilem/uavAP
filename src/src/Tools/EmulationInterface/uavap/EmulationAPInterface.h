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
#include <uavAP/Core/DataPresentation/Content.h>
#include <uavAP/Core/DataPresentation/APDataPresentation/APDataPresentation.h>
#include <uavAP/Core/IDC/Serial/SerialIDC.h>
#include <uavAP/Core/SensorData.h>
#include <uavAP/Core/Scheduler/IScheduler.h>

class EmulationAPInterface : public IAggregatableObject, public IRunnableObject
{
public:

	EmulationAPInterface();

	~EmulationAPInterface();

	static std::shared_ptr<EmulationAPInterface>
	create(const boost::property_tree::ptree& config);

	bool
	configure(const boost::property_tree::ptree& config);

	void
	notifyAggregationOnUpdate(Aggregator& agg) override;

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
	ObjectHandle<IDataPresentation<Content,Target>> dataPresentation_;
	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<IInterDeviceComm> idc_;

	std::string serialPort_;
	Sender actuationSender_;

	const int numChannels_;
	uint32_t lastSequenceNr_;
};

#endif /* AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_H_ */
