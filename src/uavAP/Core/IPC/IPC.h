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
 * IPC.h
 *
 *  Created on: Jul 18, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_IPC_IPC_H_
#define UAVAP_CORE_IPC_IPC_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/thread/lock_types.hpp>
#include <boost/interprocess/sync/sharable_lock.hpp>
#include <boost/property_tree/ptree.hpp>

#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/Core/IPC/detail/ISubscriptionImpl.h"
#include "uavAP/Core/IPC/detail/MessageQueuePublisherImpl.h"
#include "uavAP/Core/IPC/detail/MessageQueueSubscriptionImpl.h"
#include "uavAP/Core/IPC/detail/SharedMemoryPublisherImpl.h"
#include "uavAP/Core/IPC/detail/SharedMemorySubscriptionImpl.h"
#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/IPC/Subscription.h"
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/Scheduler/IScheduler.h"

class IPC: public IRunnableObject, public IAggregatableObject
{

public:

	static constexpr TypeId typeId = "ipc";


	IPC();

	~IPC();

	static std::shared_ptr<IPC>
	create(const boost::property_tree::ptree& config);

	template<class Object>
	Subscription
	subscribeOnSharedMemory(std::string id, const boost::function<void
	(const Object&)>& slot);

	template<class Object>
	Subscription
	subscribeOnMessageQueue(std::string id, const boost::function<void
	(const Object&)>& slot);

	Subscription
	subscribeOnPacket(std::string id, const boost::function<void
	(const Packet&)>& slot);

	template<class Object>
	Publisher
	publishOnSharedMemory(std::string id);

	template<class Object>
	Publisher
	publishOnMessageQueue(std::string id, unsigned int numOfMessages);

	Publisher
	publishPackets(std::string id);

	bool
	run(RunStage stage) override;

	void
	notifyAggregationOnUpdate(const Aggregator& agg);

	void
	sigintHandler(int sig);

private:

	std::vector<std::shared_ptr<IPublisherImpl>> publications_;

	std::mutex subscribeMutex_;
	std::map<std::string, std::shared_ptr<ISubscriptionImpl>> subscriptions_;

	ObjectHandle<IScheduler> scheduler_;

	bool subscribedOnSigint_;

	std::size_t packetSize_;
	unsigned int packetQueueSize_;
};

template<class Object>
inline Subscription
IPC::subscribeOnSharedMemory(std::string id, const boost::function<void
(const Object&)>& slot)
{
	std::shared_ptr<SharedMemorySubscriptionImpl<Object>> impl;

	std::lock_guard<std::mutex> lock(subscribeMutex_);

	auto subscription = subscriptions_.find(id);
	if (subscription != subscriptions_.end())
	{
		impl = std::dynamic_pointer_cast<SharedMemorySubscriptionImpl<Object>>(
				subscription->second);
		if (!impl)
		{
			APLOG_ERROR << "Subscription id found for another subscription method.";
			return Subscription();
		}
		auto con = impl->subscribe(slot);
		return Subscription(subscription->second, con);
	}
	else
	{
		try
		{
			impl = std::make_shared<SharedMemorySubscriptionImpl<Object>>(id);
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
		sched->schedule(std::bind(&SharedMemorySubscriptionImpl<Object>::start, impl),
				Milliseconds(0));
		return Subscription(std::static_pointer_cast<ISubscriptionImpl>(impl), con);
	}

	APLOG_ERROR << "Scheduler missing. Cannot start subscription.";
	return Subscription(std::static_pointer_cast<ISubscriptionImpl>(impl), con);
}

template<class Object>
inline Publisher
IPC::publishOnSharedMemory(std::string id)
{
	auto impl = std::make_shared<SharedMemoryPublisherImpl<Object>>(id, Object());
	publications_.push_back(impl);
	return Publisher(impl);
}

template<class Object>
Subscription
IPC::subscribeOnMessageQueue(std::string id, const boost::function<void
(const Object&)>& slot)
{
	std::shared_ptr<MessageQueueSubscriptionImpl<Object>> impl;

	std::lock_guard<std::mutex> lock(subscribeMutex_);

	auto subscription = subscriptions_.find(id);
	if (subscription != subscriptions_.end())
	{
		impl = std::dynamic_pointer_cast<MessageQueueSubscriptionImpl<Object>>(
				subscription->second);
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
			impl = std::make_shared<MessageQueueSubscriptionImpl<Object>>(id);
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
		sched->schedule(std::bind(&MessageQueueSubscriptionImpl<Object>::start, impl),
				Milliseconds(0));
		return Subscription(std::static_pointer_cast<ISubscriptionImpl>(impl), con);
	}

	APLOG_ERROR << "Scheduler missing. Cannot start subscription.";
	return Subscription(std::static_pointer_cast<ISubscriptionImpl>(impl), con);
}

template<class Object>
Publisher
IPC::publishOnMessageQueue(std::string id, unsigned int numOfMessages)
{
	std::shared_ptr<MessageQueuePublisherImpl<Object>> impl;
	try
	{
		impl = std::make_shared<MessageQueuePublisherImpl<Object>>(id, numOfMessages,
				sizeof(VectorWrapper<Object> ));
	} catch (boost::interprocess::interprocess_exception&)
	{
		APLOG_WARN << "Message queue with id: " << id << " already exists. Creating new.";
		boost::interprocess::message_queue::remove(id.c_str());
		impl = std::make_shared<MessageQueuePublisherImpl<Object>>(id, numOfMessages,
				sizeof(VectorWrapper<Object> ));
	}

	publications_.push_back(impl);
	return Publisher(impl);
}

#endif /* UAVAP_CORE_IPC_IPC_H_ */
