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
#include <uavAP/Core/DataPresentation/Packet.h>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Runner/IRunnableObject.h>
#include <uavAP/Core/SensorData.h>


struct ControllerOutputEdu;
class INetworkLayer;
class DataPresentation;
class SignalHandler;
class IDC;

class EduInterface: public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "edu";

	EduInterface();

	~EduInterface();

	static std::shared_ptr<EduInterface>
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
	sendActuation(const ControllerOutputEdu& control);

	void
	vector3ToArray(const Vector3& vec, double (&array)[3]);

	void
	sigHandler(int sig);

	bool setup_;
	bool subscribedOnSigint_;

	ObjectHandle<DataPresentation> dataPresentation_;
	ObjectHandle<IDC> idc_;
	ObjectHandle<SignalHandler> signalHandler_;
};

#endif /* AUTOPILOT_INTERFACE_INCLUDE_AUTOPILOT_INTERFACE_AUTOPILOTINTERFACE_H_ */
