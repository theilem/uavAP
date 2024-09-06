/*
 * FlightControlData.h
 *
 *  Created on: Jun 29, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_FLIGHTCONTROLDATA_H_
#define UAVAP_FLIGHTCONTROL_FLIGHTCONTROLDATA_H_

#include <boost/signals2.hpp>

#include <cpsCore/cps_object>
#include <cpsCore/Utilities/LockTypes.hpp>
#include <cpsCore/Utilities/IPC/Subscription.h>
#include <cpsCore/Utilities/IPC/Publisher.h>

#include "uavAP/Core/SensorData.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingIO.h"
#include "uavAP/FlightControl/SensingActuationIO/IActuationIO.h"

struct AdvancedControl;

class IPC;
class DataHandling;

class SensingActuationIO : public ISensingIO, public IActuationIO,
						   public AggregatableObject<IPC, DataHandling, ITimeProvider>,
						   public IRunnableObject
{
public:

	static constexpr TypeId typeId = "shared_memory";

	SensingActuationIO() = default;

	bool
	run(RunStage stage) override;

	void
	setControllerOutput(const ControllerOutput& out) override;

	SensorData
	getSensorData() const override;

	boost::signals2::connection
	subscribeOnSensorData(const OnSensorData::slot_type& slot) override;

private:

	void
	onSensorData(const SensorData& data);

	void
	onAdvancedControl(const AdvancedControl& control);

	Subscription sensorSubscription_;
	Subscription powerSubscription_;
	Subscription servoSubscription_;
	Publisher<ControllerOutput> actuationPublisher_;
	Publisher<AdvancedControl> advancedControlPublisher_;

	OnSensorData onSensorData_;

	mutable SharedMutex mutex_;
	SensorData sensorData_;
	PowerData powerData_;
	ServoData servoData_;
	mutable SharedMutex controllerOutputMutex_;
	TimedValue<ControllerOutput> controllerOutput_;

};

#endif /* UAVAP_FLIGHTCONTROL_FLIGHTCONTROLDATA_H_ */
