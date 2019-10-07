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
 * SerialIDC.cpp
 *
 *  Created on: Jul 31, 2017
 *      Author: mircot
 */
#include <uavAP/Core/Object/AggregatableObjectImpl.hpp>
#include "uavAP/Core/IDC/NetworkLayer/Serial/SerialNetworkLayer.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/Scheduler/IScheduler.h"
#include "uavAP/Core/Object/SignalHandler.h"

std::shared_ptr<INetworkLayer>
SerialNetworkLayer::create(const Configuration& config)
{
	auto snl = std::make_shared<SerialNetworkLayer>();
	snl->configure(config);
	return snl;
}

bool
SerialNetworkLayer::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);
	Configuration ports;
	pm.add("ports", ports, true);

	for (const auto& it : ports)
	{
		SerialNetworkParams params;
		if (!params.configure(it.second))
			return false;

		handler_.emplace(it.first, std::make_shared<SerialHandler>(params));
	}
	return pm.map();
}

bool
SerialNetworkLayer::sendPacket(const std::string& id, const Packet& packet)
{
	auto it = handler_.find(id);
	if (it == handler_.end())
	{
		APLOG_ERROR << "Connection id not specified: " << id;
		return false;
	}

	return it->second->sendPacket(packet);
}

boost::signals2::connection
SerialNetworkLayer::subscribeOnPacket(const std::string& id, const OnPacket::slot_type& handle)
{
	auto it = handler_.find(id);
	if (it == handler_.end())
	{
		APLOG_ERROR << "Connection id not specified: " << id;
		return boost::signals2::connection();
	}

	return it->second->subscribeOnPackets(handle);
}

bool
SerialNetworkLayer::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!checkIsSet<IScheduler, SignalHandler>())
		{
			APLOG_ERROR << "SerialNetworkLayer missing Dependencies";
			return true;
		}
		break;
	}
	case RunStage::NORMAL:
	{
		auto sched = get<IScheduler>();
		for (const auto& it : handler_)
		{
			sched->schedule(std::bind(&SerialHandler::startHandler, it.second), Milliseconds(0));
		}

		if (auto sh = get<SignalHandler>())
		{
 			sh->subscribeOnSigint(
					std::bind(&SerialNetworkLayer::onSigInt, this, std::placeholders::_1));
		}
		break;
	}
	default:
		break;
	}
	return false;
}

void
SerialNetworkLayer::onSigInt(int sig)
{
	if (sig == SIGINT || sig == SIGTERM)
	{
		APLOG_DEBUG << "Caught sigint. Canceling Serial handlers.";
		for (const auto& it : handler_)
		{
			it.second->cancelHandler();
		}
		handler_.clear();
	}
}

SerialNetworkLayer::~SerialNetworkLayer()
{
}

