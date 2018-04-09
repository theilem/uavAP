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

#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/SensorData.h"
#include <boost/thread/shared_mutex.hpp>
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingActuationIO.h"

#include <boost/signals2.hpp>

struct ControllerOutput;

class SensingActuationIO: public ISensingActuationIO, public IAggregatableObject, public IRunnableObject
{
public:

	SensingActuationIO();

	static std::shared_ptr<SensingActuationIO>
	create(const boost::property_tree::ptree& config);

	void
	notifyAggregationOnUpdate(Aggregator& agg) override;

	bool
	run(RunStage stage) override;

	void
	setControllerOutput(const ControllerOutput& out);

	using OnSensorData = boost::signals2::signal<void(const SensorData&)>;

	boost::signals2::connection
	subscribeOnSensorData(const OnSensorData::slot_type& slot);

	boost::shared_mutex mutex;
	SensorData sensorData;

private:

	void
	onSensorData(const SensorData& data);

	ObjectHandle<IPC> ipc_;

	Subscription sensorSubscription_;
	Publisher actuationPublisher_;

	OnSensorData onSensorData_;

};

#endif /* UAVAP_FLIGHTCONTROL_FLIGHTCONTROLDATA_H_ */
