/*
 * FlightControlData.h
 *
 *  Created on: Jun 29, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_SENSINGACTUATIONIO_SENSINGIO_H_
#define UAVAP_FLIGHTCONTROL_SENSINGACTUATIONIO_SENSINGIO_H_

#include <boost/signals2.hpp>

#include <cpsCore/cps_object>
#include <cpsCore/Utilities/LockTypes.hpp>
#include <cpsCore/Utilities/IPC/Subscription.h>

#include "uavAP/Core/SensorData.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingIO.h"

class IPC;

class SensingIO : public ISensingIO,
				  public AggregatableObject<IPC>,
				  public IRunnableObject
{
public:

	static constexpr TypeId typeId = "shared_memory";

	SensingIO() = default;

	bool
	run(RunStage stage) override;

	SensorData
	getSensorData() const override;

	boost::signals2::connection
	subscribeOnSensorData(const OnSensorData::slot_type& slot) override;

private:

	void
	onSensorData(const SensorData& data);

	Subscription sensorSubscription_;
	Subscription powerSubscription_;
	Subscription servoSubscription_;

	OnSensorData onSensorData_;

	mutable SharedMutex mutex_;
	SensorData sensorData_;
	PowerData powerData_;
	ServoData servoData_;

};

#endif /* UAVAP_FLIGHTCONTROL_SENSINGACTUATIONIO_SENSINGIO_H_ */
