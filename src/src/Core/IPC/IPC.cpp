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
 * IPC.cpp
 *
 *  Created on: Jul 18, 2017
 *      Author: mircot
 */

#include "uavAP/Core/IPC/detail/PacketPublisherImpl.h"
#include "uavAP/Core/IPC/detail/PacketSubscriptionImpl.h"
#include "uavAP/Core/IPC/IPC.h"
#include <csignal>

IPC::IPC() :
		subscribedOnSigint_(false), packetSize_(16000), packetQueueSize_(10)
{
}

IPC::~IPC()
{
	std::lock_guard<std::mutex> lock(subscribeMutex_);
	for (auto& it : subscriptions_)
	{
		it.second->cancel();
	}
}

std::shared_ptr<IPC>
IPC::create(const boost::property_tree::ptree&)
{
	return std::make_shared<IPC>();
}

bool
IPC::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
		if (!scheduler_.isSet())
		{
			APLOG_WARN << "IPC is missing Scheduler. Retry functionality deactivated.";
		}
		break;
	default:
		break;
	}
	return false;
}

void
IPC::notifyAggregationOnUpdate(const Aggregator& agg)
{
	scheduler_.setFromAggregationIfNotSet(agg);

	if (!subscribedOnSigint_)
	{
		agg.subscribeOnSigint(std::bind(&IPC::sigintHandler, this, std::placeholders::_1));
		subscribedOnSigint_ = true;
	}
}

Subscription
IPC::subscribeOnPacket(std::string id, const boost::function<void
(const Packet&)>& slot)
{
	std::shared_ptr<PacketSubscriptionImpl> impl;

	std::lock_guard<std::mutex> lock(subscribeMutex_);

	auto subscription = subscriptions_.find(id);
	if (subscription != subscriptions_.end())
	{
		impl = std::dynamic_pointer_cast<PacketSubscriptionImpl>(subscription->second);
		if (!impl)
		{
			APLOG_ERROR << "Subscription id found for another subscription method.";
			return Subscription();
		}
	}
	else
	{
		try
		{
			impl = std::make_shared<PacketSubscriptionImpl>(id, packetSize_);
			subscriptions_.insert(
					std::make_pair(id, std::static_pointer_cast<ISubscriptionImpl>(impl)));
		} catch (boost::interprocess::interprocess_exception&)
		{
			return Subscription();
		}
	}

	auto con = impl->subscribe(slot);

	if (auto sched = scheduler_.get())
	{
		sched->schedule(std::bind(&PacketSubscriptionImpl::start, impl), Milliseconds(0));
		return Subscription(std::static_pointer_cast<ISubscriptionImpl>(impl), con);
	}

	APLOG_ERROR << "Scheduler missing. Cannot start subscription.";
	return Subscription(std::static_pointer_cast<ISubscriptionImpl>(impl), con);
}

Publisher
IPC::publishPackets(std::string id)
{
	std::shared_ptr<PacketPublisherImpl> impl;
	try
	{
		impl = std::make_shared<PacketPublisherImpl>(id, packetQueueSize_, packetSize_);
	} catch (boost::interprocess::interprocess_exception&)
	{
		APLOG_WARN << "Message queue with id: " << id << " already exists. Creating new.";
		boost::interprocess::message_queue::remove(id.c_str());
		impl = std::make_shared<PacketPublisherImpl>(id, packetQueueSize_, packetSize_);
	}

	publications_.push_back(impl);
	return Publisher(impl);
}

void
IPC::sigintHandler(int sig)
{
	if (sig == SIGINT || sig == SIGTERM)
	{
		APLOG_DEBUG << "Caught sigint. Deleting subscriptions and publications.";
		subscriptions_.clear();
		publications_.clear();
	}
	exit(sig);
}
