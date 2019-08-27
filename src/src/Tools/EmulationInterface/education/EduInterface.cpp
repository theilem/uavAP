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
 * AutopilotInterface.cpp
 *
 *  Created on: Nov 25, 2017
 *      Author: mircot
 */

#include "EduInterface.h"
#include "api_edu.h"
#include "ControllerOutputEdu.h"
#include "SensorDataEdu.h"
#include <uavAP/Core/DataPresentation/Content.h>
#include <uavAP/Core/IDC/IDC.h>
#include <uavAP/Core/IDC/NetworkLayer/INetworkLayer.h>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/Object/SignalHandler.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>
#include <uavAP/Core/DataPresentation/DataPresentation.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>


EduInterface::EduInterface() :
		setup_(false), subscribedOnSigint_(false)
{
}

EduInterface::~EduInterface()
{
	if (api_terminate() != 0)
	{
		APLOG_ERROR
				<< "Shutdown unsuccessful. Check for possible memory leakage or zombie processes.";
	}
}

std::shared_ptr<EduInterface>
EduInterface::create(const Configuration& config)
{
	auto emulation = std::make_shared<EduInterface>();
	if (!emulation->configure(config))
		APLOG_ERROR << "Configuration of EduInterface not successfull.";
	return emulation;
}

bool
EduInterface::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);
	return pm.map();
}

void
EduInterface::notifyAggregationOnUpdate(const Aggregator& agg)
{
	idc_.setFromAggregationIfNotSet(agg);
	dataPresentation_.setFromAggregationIfNotSet(agg);
	signalHandler_.setFromAggregationIfNotSet(agg);
}

bool
EduInterface::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!idc_.isSet())
		{
			APLOG_ERROR << "EduInterface idc missing.";
			return true;
		}
		if (!dataPresentation_.isSet())
		{
			APLOG_ERROR << "EduInterface dataPresentation missing.";
			return true;
		}
		if (auto sh = signalHandler_.get())
		{
			sh->subscribeOnSigint(std::bind(&EduInterface::sigHandler, this, std::placeholders::_1));
			subscribedOnSigint_ = true;
		}

		int result = api_initialize();
		if (result != 0)
		{
			APLOG_ERROR << "Failed to initialize the API. Result: " << result << "; Abort.";
			return true;
		}
		setup_ = true;
		break;
	}
	case RunStage::NORMAL:
	{
		auto idc = idc_.get();
		idc->subscribeOnPacket("sim",
				std::bind(&EduInterface::onPacket, this, std::placeholders::_1));

		break;
	}
	case RunStage::FINAL:
		break;
	default:
		break;
	}
	return false;
}

void
EduInterface::onSensorData(const SensorData& sensorData)
{
	if (!setup_)
	{
		APLOG_ERROR << "ApExt not setup. Cannot send sensor data through api.";
		return;
	}

	//First get the most recent actuation
	ControllerOutputEdu eduControl;
	int actuationResult = api_actuate(eduControl);
	if (actuationResult == 0)
	{
		sendActuation(eduControl);
	}
	else
	{
		APLOG_WARN << "Actuation unsuccessful. Ignore command.";
	}

	SensorDataEdu eduSense;

	vector3ToArray(sensorData.position, eduSense.position);
	vector3ToArray(sensorData.velocity, eduSense.velocity);
	vector3ToArray(sensorData.acceleration, eduSense.acceleration);
	vector3ToArray(sensorData.attitude, eduSense.attitude);
	vector3ToArray(sensorData.angularRate, eduSense.angularRate);
	eduSense.sequenceNr = sensorData.sequenceNr;

	int sensResult = api_sense(eduSense);

	if (sensResult != 0)
	{
		APLOG_WARN << "Something went wrong sending the sensor data.";
	}
}

void
EduInterface::onPacket(const Packet& packet)
{
	APLOG_DEBUG << "Received packet.";
	auto dp = dataPresentation_.get();
	if (!dp)
	{
		APLOG_ERROR << "Data presentation missing. Cannot deserialize packet.";
		return;
	}

	auto p = packet;
	Content content = dp->extractHeader<Content>(p);

	if (content == Content::SENSOR_DATA)
	{
		onSensorData(dp->deserialize<SensorData>(p));
	}
	else
	{
		APLOG_ERROR
				<< "Invalid packet received. Only sensor data allowed. Content: "
				<< static_cast<int>(content);
		return;
	}

}

void
EduInterface::sendActuation(const ControllerOutputEdu& control)
{
	ControllerOutput out;
	out.pitchOutput = control.pitchOutput;
	out.rollOutput = control.rollOutput;
	out.throttleOutput = 2 * control.throttleOutput - 1;
	out.yawOutput = control.yawOutput;
	out.sequenceNr = control.sequenceNr;

	auto dp = dataPresentation_.get();
	if (!dp)
	{
		APLOG_ERROR << "Data presentation missing. Cannot send Actuation.";
		return;
	}
	auto packet = dp->serialize(out);
	dp->addHeader(packet, Content::CONTROLLER_OUTPUT);

	auto idc = idc_.get();
	if (!idc)
	{
		APLOG_ERROR << "Network missing. Cannot send Actuation.";
		return;
	}
	idc->sendPacket("sim", packet);
}

void
EduInterface::vector3ToArray(const Vector3& vec, double (&array)[3])
{
	array[0] = vec[0];
	array[1] = vec[1];
	array[2] = vec[2];
}

void
EduInterface::sigHandler(int sig)
{
	api_terminate();
}
