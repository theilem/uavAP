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
 * SharedMemorySubscriptionImpl.h
 *
 *  Created on: Aug 3, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_IPC_DETAIL_SHAREDMEMORYSUBSCRIPTIONIMPL_H_
#define UAVAP_CORE_IPC_DETAIL_SHAREDMEMORYSUBSCRIPTIONIMPL_H_
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/signals2.hpp>
#include <boost/thread/pthread/thread_data.hpp>
#include <boost/thread/thread_time.hpp>
#include "uavAP/Core/IPC/detail/ISubscriptionImpl.h"
#include "uavAP/Core/IPC/detail/MessageObject.h"
#include "uavAP/Core/Time.h"
#include <thread>

template<class Object>
class SharedMemorySubscriptionImpl: public ISubscriptionImpl
{
public:

	SharedMemorySubscriptionImpl(std::string id);

	~SharedMemorySubscriptionImpl();

	void
	cancel() override;

	void
	start() override;

	using OnSharedMem = boost::signals2::signal<void(const Object&)>;
	using OnSharedMemSlot = boost::function<void(const Object&)>;

	boost::signals2::connection
	subscribe(const OnSharedMemSlot& slot);

private:

	void
	onSharedMemory();

	boost::interprocess::shared_memory_object sharedMem_;

	OnSharedMem onSharedMem_;

	std::thread listenerThread_;

	std::atomic_bool listenerCanceled_;
};

template<class Object>
inline
SharedMemorySubscriptionImpl<Object>::SharedMemorySubscriptionImpl(std::string id) :
		sharedMem_(boost::interprocess::open_only, id.c_str(), boost::interprocess::read_write), listenerCanceled_(
				false)
{
}

template<class Object>
inline boost::signals2::connection
SharedMemorySubscriptionImpl<Object>::subscribe(const OnSharedMemSlot& slot)
{
	return onSharedMem_.connect(slot);
}

template<class Object>
inline void
SharedMemorySubscriptionImpl<Object>::cancel()
{
	listenerCanceled_.store(true);
}

template<class Object>
inline
SharedMemorySubscriptionImpl<Object>::~SharedMemorySubscriptionImpl()
{
	if (!listenerCanceled_.load())
	{
		cancel();
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); //Wait for timeout of condition to stop subscription

	std::string name = sharedMem_.get_name();
//	if (sharedMem_.remove(name.c_str()))
//		APLOG_DEBUG << name << " shared memory removed.";
}

template<class Object>
inline void
SharedMemorySubscriptionImpl<Object>::start()
{
	listenerThread_ = std::thread(
			std::bind(&SharedMemorySubscriptionImpl<Object>::onSharedMemory, this));
	listenerThread_.detach();
}

template<class Object>
inline void
SharedMemorySubscriptionImpl<Object>::onSharedMemory()
{
	using namespace boost::interprocess;
	mapped_region region(sharedMem_, read_write);
	MessageObject<Object>* message = static_cast<MessageObject<Object>*>(region.get_address());

	boost::interprocess::sharable_lock<boost::interprocess::interprocess_sharable_mutex> lock(
			message->mtx);
	for (;;)
	{
		if (listenerCanceled_.load())
		{
			return;
		}

		auto timeout = boost::get_system_time() + boost::posix_time::milliseconds(100);
		if (!message->cnd.timed_wait(lock, timeout))
		{
			continue;
		}
		Object obj = *static_cast<Object*>(message);
		onSharedMem_(obj);
	}
}

#endif /* UAVAP_CORE_IPC_DETAIL_SHAREDMEMORYSUBSCRIPTIONIMPL_H_ */
