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
 * MessageQueueSubscriptionImpl.h
 *
 *  Created on: Aug 3, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_IPC_DETAIL_MESSAGEQUEUESUBSCRIPTIONIMPL_H_
#define UAVAP_CORE_IPC_DETAIL_MESSAGEQUEUESUBSCRIPTIONIMPL_H_
#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/signals2.hpp>
#include <boost/thread/thread_time.hpp>
#include "uavAP/Core/IPC/detail/ISubscriptionImpl.h"
#include "uavAP/Core/IPC/detail/MessageObject.h"
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/Time.h"
#include <thread>

template <class Object>
class MessageQueueSubscriptionImpl: public ISubscriptionImpl
{
public:

	MessageQueueSubscriptionImpl(const std::string& id);

	~MessageQueueSubscriptionImpl();

	void
	cancel() override;

	void
	start() override;

	using OnMessageQueue = boost::signals2::signal<void(const Object&)>;
	using OnMessageQueueSlot = boost::function<void(const Object&)>;

	boost::signals2::connection
	subscribe(const OnMessageQueueSlot& slot);

private:

	void
	onMessageQueue();

	boost::interprocess::message_queue messageQueue_;

	OnMessageQueue onMesssageQueue_;

	std::thread listenerThread_;

	std::atomic_bool listenerCanceled_;

	std::string id_;
};

template<class Object>
inline
MessageQueueSubscriptionImpl<Object>::MessageQueueSubscriptionImpl(const std::string& id):
	messageQueue_(boost::interprocess::open_only, id.c_str()),
	listenerCanceled_(false),
	id_(id)
{
}

template<class Object>
inline
MessageQueueSubscriptionImpl<Object>::~MessageQueueSubscriptionImpl()
{
	if (!listenerCanceled_.load())
	{
		cancel();
	}
	boost::this_thread::sleep(Milliseconds(100)); //Wait for timeout of message_queue to stop subscription
//	if (messageQueue_.remove(id_.c_str()))
//		APLOG_DEBUG << id_ << " message queue removed.";
}

template<class Object>
inline void
MessageQueueSubscriptionImpl<Object>::cancel()
{
	listenerCanceled_.store(true);
}

template<class Object>
inline void
MessageQueueSubscriptionImpl<Object>::start()
{
	listenerThread_ = std::thread(std::bind(&MessageQueueSubscriptionImpl<Object>::onMessageQueue, this));
	listenerThread_.detach();
}

template<class Object>
inline boost::signals2::connection
MessageQueueSubscriptionImpl<Object>::subscribe(const OnMessageQueueSlot& slot)
{
	return onMesssageQueue_.connect(slot);
}

template<class Object>
inline void
MessageQueueSubscriptionImpl<Object>::onMessageQueue()
{
	using namespace boost::interprocess;
	unsigned long expectedSize = sizeof(Object);
	unsigned long maxSize = sizeof(VectorWrapper<Object> );

	for (;;)
	{
		if (listenerCanceled_.load())
		{
			return;
		}
		Object object;
		void* buffer = &object;
		message_queue::size_type size;
		unsigned int priority;
		auto timeout = boost::get_system_time() + Milliseconds(100);
		if (!messageQueue_.timed_receive(buffer, maxSize, size, priority, timeout))
		{
			continue;
		}
		if (expectedSize != size)
		{
			APLOG_ERROR << "Invalid message received";
			continue;
		}
		onMesssageQueue_(object);
	}
}

#endif /* UAVAP_CORE_IPC_DETAIL_MESSAGEQUEUESUBSCRIPTIONIMPL_H_ */
