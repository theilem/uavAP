/*
 * AutopilotInterface.h
 *
 *  Created on: Nov 25, 2017
 *      Author: mircot
 */

#ifndef AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_H_
#define AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_H_
#include <boost/property_tree/ptree.hpp>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Runner/IRunnableObject.h>

#include <uavAP/Core/DataPresentation/Content.h>
#include <uavAP/Core/DataPresentation/APDataPresentation/APDataPresentation.h>
#include <uavAP/Core/IDC/Serial/SerialIDC.h>
#include <uavAP/Core/Scheduler/IScheduler.h>

struct ControllerOutputEdu;

class EduInterface : public IAggregatableObject, public IRunnableObject
{
public:

	EduInterface();

	~EduInterface();

	static std::shared_ptr<EduInterface>
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
	sendActuation(const ControllerOutputEdu& control);

	void
	vector3ToArray(const Vector3& vec, double (&array)[3]);

    void
    sigHandler(int sig);

	bool setup_;
    bool subscribedOnSigint_;

	ObjectHandle<IDataPresentation<Content,Target>> dataPresentation_;
	ObjectHandle<IInterDeviceComm> idc_;

	std::string serialPort_;
	Sender actuationSender_;

};

#endif /* AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_H_ */
