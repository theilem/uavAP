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
 * FlightControlData.h
 *
 *  Created on: Jun 29, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_FLIGHTCONTROLDATA_H_
#define UAVAP_FLIGHTCONTROL_FLIGHTCONTROLDATA_H_

#include <boost/thread/shared_mutex.hpp>
#include <boost/signals2.hpp>
#include <boost/property_tree/ptree.hpp>

#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/IPC/Subscription.h"
#include "uavAP/Core/SensorData.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingActuationIO.h"


struct ControllerOutput;
class IPC;

class SensingActuationIO: public ISensingActuationIO,
		public IAggregatableObject,
		public IRunnableObject
{
public:

	static constexpr TypeId typeId = "shared_memory";

	SensingActuationIO();

	ADD_CREATE_WITHOUT_CONFIG(SensingActuationIO)

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

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

	ObjectHandle<IPC> ipc_;

	Subscription sensorSubscription_;
	Publisher actuationPublisher_;

	OnSensorData onSensorData_;

	mutable boost::shared_mutex mutex_;
	SensorData sensorData_;

};

#endif /* UAVAP_FLIGHTCONTROL_FLIGHTCONTROLDATA_H_ */
