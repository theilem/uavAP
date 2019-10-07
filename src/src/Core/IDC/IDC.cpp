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
 * IDC.cpp
 *
 *  Created on: Jul 28, 2018
 *      Author: mircot
 */
#include <uavAP/Core/IDC/IDC.h>
#include <uavAP/Core/IDC/NetworkLayer/INetworkLayer.h>
#include <uavAP/Core/IDC/TransportLayer/ITransportLayer.h>
#include <uavAP/Core/Logging/APLogger.h>

std::shared_ptr<IDC>
IDC::create(const Configuration&)
{
	return std::make_shared<IDC>();
}

void
IDC::notifyAggregationOnUpdate(const Aggregator& agg)
{
	network_.setFromAggregationIfNotSet(agg);
	transport_.setFromAggregationIfNotSet(agg);
	self_.setFromAggregationIfNotSet(agg);
}

bool
IDC::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!self_.isSet())
		{
			APLOG_ERROR << "Something went really wrong";
			return true;
		}
		if (!transport_.isSet())
		{
			if (!network_.isSet())
			{
				APLOG_ERROR << "Transport and Network layer missing. IDC needs one.";
				return true;
			}
		}
		break;
	}
	default:
		break;
	}
	return false;
}

bool
IDC::sendPacket(const std::string& id, const Packet& packet, bool ack)
{
	auto transport = transport_.get();

	if (!transport)
	{
		auto network = network_.get();
		if (!network)
		{
			APLOG_ERROR << "Transport and Network layer missing. IDC needs one.";
			return false;
		}

		if (ack)
		{
			APLOG_WARN << "Acknowledgment cannot be requested because TransportLayer is missing";
		}

		return network->sendPacket(id, packet);
	}
	return transport->send(id, packet, ack);
}

boost::signals2::connection
IDC::subscribeOnPacket(const std::string& id, const OnPacket::slot_type& handle)
{
	auto transport = transport_.get();

	if (!transport)
	{
		auto network = network_.get();
		if (!network)
		{
			APLOG_ERROR << "Transport and Network layer missing. IDC needs one.";
			return boost::signals2::connection();
		}

		return network->subscribeOnPacket(id, handle);
	}
	return transport->subscribeOnPacket(id, handle);
}

IDCSender
IDC::createSender(const std::string& id)
{
	auto self = self_.get();
	if (!self)
	{
		APLOG_ERROR << "Something went terribly wrong";
		return IDCSender();
	}

	return IDCSender(self, id);
}
