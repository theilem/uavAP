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

#include <uavAP/Core/Object/SignalHandler.h>
#include <uavAP/Core/Scheduler/IScheduler.h>
#include "uavAP/Core/IPC/IPC.h"
#include <uavAP/Core/PropertyMapper/ConfigurableObjectImpl.hpp>
#include <csignal>

IPC::IPC()
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

bool
IPC::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!isSet<IScheduler>())
		{
			APLOG_WARN << "IPC is missing Scheduler. Subscription functionality disabled.";
		}
		break;
	}
	case RunStage::NORMAL:
	{
		if (auto sh = get<SignalHandler>())
		{
			APLOG_DEBUG << "IPC subscribing on SIGINT";
			sh->subscribeOnSigint(std::bind(&IPC::sigintHandler, this, std::placeholders::_1));
		}

		if (params.retryPeriod() != 0)
		{
			if (auto sched = get<IScheduler>())
			{
				APLOG_DEBUG << "Schedule retries";
				sched->schedule(std::bind(&IPC::retrySubscriptions, this), Milliseconds(0),
						Milliseconds(params.retryPeriod()));
			}
		}
		break;
	}
	default:
		break;
	}
	return false;
}

Publisher<Packet>
IPC::publishPackets(const std::string& id, const IPCOptions& options)
{
	if (options.multiTarget)
	{
		return Publisher<Packet>(publishOnSharedMemory(id, params.maxPacketSize()),
				[](const Packet& p)
				{	return p;});
	}
	else
	{
		return Publisher<Packet>(
				publishOnMessageQueue(id, params.maxPacketSize(), params.maxNumPackets()),
				[](const Packet& p)
				{	return p;});
	}
}

std::shared_ptr<IPublisherImpl>
IPC::publishOnSharedMemory(const std::string& id, unsigned maxPacketSize)
{
	auto impl = std::make_shared<SharedMemoryPublisherImpl>(id, maxPacketSize);
	publications_.push_back(impl);
	return impl;
}

std::shared_ptr<IPublisherImpl>
IPC::publishOnMessageQueue(const std::string& id, unsigned maxPacketSize, unsigned numPackets)
{
	std::shared_ptr<MessageQueuePublisherImpl> impl;
	try
	{
		impl = std::make_shared<MessageQueuePublisherImpl>(id, numPackets, maxPacketSize);
	} catch (boost::interprocess::interprocess_exception&)
	{
		APLOG_WARN << "Message queue with id: " << id << " already exists. Creating new.";
		boost::interprocess::message_queue::remove(id.c_str());
		impl = std::make_shared<MessageQueuePublisherImpl>(id, numPackets, maxPacketSize);
	}

	publications_.push_back(impl);
	return impl;
}

void
IPC::sigintHandler(int sig)
{
	if (sig == SIGINT || sig == SIGTERM)
	{
		APLOG_DEBUG << "Caught signal " << sig << ". Deleting subscriptions and publications.";
		APLogger::instance()->flush();
		for (const auto& sub : subscriptions_)
		{
			sub.second->cancel();
		}

		subscriptions_.clear();
		publications_.clear();
	}
}

Subscription
IPC::subscribeOnPackets(const std::string& id, const std::function<void
(const Packet&)>& slot, const IPCOptions& options)
{
	Subscription result;
	if (options.multiTarget)
	{
		result = subscribeOnSharedMemory(id, slot);
	}
	else
	{
		result = subscribeOnMessageQueue(id, slot);
	}

	if (!result.connected() && options.retry)
	{
		LockGuard l(retryVectorMutex_);
		APLOG_DEBUG << "Queuing " << id << " for retry";
		//Schedule retry
		if (options.multiTarget)
		{
			retryVector_.push_back(
					std::make_pair(options.retrySuccessCallback,
							std::bind(&IPC::subscribeOnSharedMemory, this, id, slot)));
		}
		else
		{
			retryVector_.push_back(
					std::make_pair(options.retrySuccessCallback,
							std::bind(&IPC::subscribeOnMessageQueue, this, id, slot)));
		}
	}

	return result;
}

Subscription
IPC::subscribeOnSharedMemory(const std::string& id, const std::function<void
(const Packet&)>& slot)
{
	std::shared_ptr<ISubscriptionImpl> impl;

	std::lock_guard<std::mutex> lock(subscribeMutex_);

	auto subscription = subscriptions_.find(id);
	if (subscription != subscriptions_.end())
	{
		impl = subscription->second;
		auto con = impl->subscribe(slot);
		return Subscription(subscription->second, con);
	}
	else
	{
		try
		{
			auto sub = std::make_shared<SharedMemorySubscriptionImpl>(id);
			if (auto sched = get<IScheduler>())
			{
				sched->schedule(std::bind(&SharedMemorySubscriptionImpl::start, sub),
						Milliseconds(0));
			}
			else
			{
				APLOG_ERROR << "Scheduler missing. Cannot start subscription.";
			}
			impl = sub;
			subscriptions_.insert(std::make_pair(id, impl));
		} catch (boost::interprocess::interprocess_exception& err)
		{
			return Subscription();
		}
	}

	auto con = impl->subscribe(slot);

	return Subscription(impl, con);
}

Subscription
IPC::subscribeOnMessageQueue(const std::string& id, const std::function<void
(const Packet&)>& slot)
{
	std::shared_ptr<ISubscriptionImpl> impl;

	std::lock_guard<std::mutex> lock(subscribeMutex_);

	auto subscription = subscriptions_.find(id);
	if (subscription != subscriptions_.end())
	{
		impl = subscription->second;
		auto con = impl->subscribe(slot);
		return Subscription(subscription->second, con);
	}
	else
	{
		try
		{
			auto sub = std::make_shared<MessageQueueSubscriptionImpl>(id, params.maxPacketSize());
			if (auto sched = get<IScheduler>())
			{
				sched->schedule(std::bind(&MessageQueueSubscriptionImpl::start, sub),
						Milliseconds(0));
			}
			else
			{
				APLOG_ERROR << "Scheduler missing. Cannot start subscription.";
			}
			impl = sub;
			subscriptions_.insert(std::make_pair(id, impl));
		} catch (boost::interprocess::interprocess_exception&)
		{
			return Subscription();
		}
	}

	auto con = impl->subscribe(slot);

	return Subscription(impl, con);
}

void
IPC::retrySubscriptions()
{
	LockGuard l(retryVectorMutex_);

	auto it = retryVector_.begin();
	while (it != retryVector_.end())
	{
		auto sub = it->second();

		if (sub.connected())
		{
			if (it->first)
				it->first(sub);
			it = retryVector_.erase(it);
		}
		else
			it++;
	}

}
