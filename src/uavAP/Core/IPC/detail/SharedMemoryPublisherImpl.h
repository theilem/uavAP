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
 * SharedMemoryPublisherImpl.h
 *
 *  Created on: Aug 2, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_IPC_SHAREDMEMORYPUBLISHERIMPL_H_
#define UAVAP_CORE_IPC_SHAREDMEMORYPUBLISHERIMPL_H_
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/thread/lock_types.hpp>
#include "uavAP/Core/IPC/detail/IPublisherImpl.h"
#include "uavAP/Core/IPC/detail/MessageObject.h"
#include "uavAP/Core/Logging/APLogger.h"

template<class Object>
class SharedMemoryPublisherImpl: public IPublisherImpl
{
public:

	SharedMemoryPublisherImpl(std::string id, Object init);

	~SharedMemoryPublisherImpl();

	void
	publish(const boost::any& obj) override;

	void
	publish(const std::vector<boost::any>& vec) override;

private:

	boost::interprocess::shared_memory_object sharedMem_;

};

template<class Object>
inline
SharedMemoryPublisherImpl<Object>::SharedMemoryPublisherImpl(std::string id, Object init)
{
	using namespace boost::interprocess;
	try
	{
		sharedMem_ = shared_memory_object(open_only, id.c_str(), read_write);
		APLOG_WARN << "Shared memory object " << id << " aready exists. Using existing.";
	} catch (interprocess_exception&)
	{
		sharedMem_ = shared_memory_object(create_only, id.c_str(), read_write);
		sharedMem_.truncate(sizeof(MessageObject<Object> ));
	}
	mapped_region region(sharedMem_, read_write);
	void* address = region.get_address();
	MessageObject<Object> message;
	*static_cast<Object*>(&message) = init;
	memcpy(address, &message, sizeof(message));
}

template<class Object>
inline void
SharedMemoryPublisherImpl<Object>::publish(const boost::any& obj)
{
	using namespace boost::interprocess;
	Object casted;
	try
	{
		casted = boost::any_cast<Object>(obj);
	} catch (boost::bad_any_cast&)
	{
		APLOG_ERROR << "Wrong data type of the Object to be published.";
		return;
	}

	mapped_region region(sharedMem_, read_write);
	MessageObject<Object>* message = static_cast<MessageObject<Object>*>(region.get_address());

	boost::unique_lock<boost::interprocess::interprocess_sharable_mutex> lock(message->mtx);
	Object* object = static_cast<Object*>(message);
	*object = casted;
	message->cnd.notify_all();
}

template<class Object>
inline
SharedMemoryPublisherImpl<Object>::~SharedMemoryPublisherImpl()
{
	std::string name = sharedMem_.get_name();
	if (sharedMem_.remove(name.c_str()))
		APLOG_DEBUG << name << " shared memory removed.";
}

template<class Object>
inline void
SharedMemoryPublisherImpl<Object>::publish(const std::vector<boost::any>& vec)
{
	APLOG_ERROR << "Cannot send vector via SharedMemory.";
}

#endif /* UAVAP_CORE_IPC_SHAREDMEMORYPUBLISHERIMPL_H_ */
