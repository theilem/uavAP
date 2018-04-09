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
 * MessageQueuePublisherImpl.h
 *
 *  Created on: Aug 3, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_IPC_MESSAGEQUEUEPUBLISHERIMPL_H_
#define UAVAP_CORE_IPC_MESSAGEQUEUEPUBLISHERIMPL_H_
#include <boost/interprocess/ipc/message_queue.hpp>
#include "uavAP/Core/IPC/detail/IPublisherImpl.h"
#include "uavAP/Core/IPC/detail/MessageObject.h"
#include "uavAP/Core/Logging/APLogger.h"

template<class Object>
class MessageQueuePublisherImpl: public IPublisherImpl
{
public:

	MessageQueuePublisherImpl(std::string id, unsigned int numOfMessages, std::size_t elementSize);

	~MessageQueuePublisherImpl();

	void
	publish(const boost::any& obj) override;

	void
	publish(const std::vector<boost::any>& vec) override;

private:

	boost::interprocess::message_queue messageQueue_;

	std::string id_;

};

template<class Object>
inline
MessageQueuePublisherImpl<Object>::MessageQueuePublisherImpl(std::string id,
		unsigned int numOfMessages, std::size_t elementSize) :
		messageQueue_(boost::interprocess::create_only, id.c_str(), numOfMessages, elementSize), id_(
				id)
{
}

template<class Object>
inline
MessageQueuePublisherImpl<Object>::~MessageQueuePublisherImpl()
{
	if (messageQueue_.remove(id_.c_str()))
		APLOG_DEBUG << id_ << " message queue removed.";
}

template<class Object>
inline void
MessageQueuePublisherImpl<Object>::publish(const boost::any& obj)
{
	Object casted;
	try
	{
		casted = boost::any_cast<Object>(obj);
	} catch (boost::bad_any_cast&)
	{
		APLOG_ERROR << "Wrong data type of the Object to be published.";
		return;
	}
	unsigned int priority = 1; //TODO Adjust priority?
	if (!messageQueue_.try_send(static_cast<const void*>(&casted), sizeof(casted), priority))
	{
		APLOG_WARN << "Message queue full. Do not send.";
	}
}

template<class Object>
inline void
MessageQueuePublisherImpl<Object>::publish(const std::vector<boost::any>& vec)
{
	try
	{
		unsigned int priority = 1; //TODO Adjust priority?
		VectorWrapper<Object> wrapper;
		Object* obj = &wrapper;
		for (auto it = vec.begin(); it != vec.end(); ++it)
		{
			*obj = boost::any_cast<Object>(*it);
			if (it == vec.begin())
				wrapper.indicator = VectorWrapperIndicator::BEGIN;
			else if (it == vec.end() - 1)
				wrapper.indicator = VectorWrapperIndicator::END;
			else
				wrapper.indicator = VectorWrapperIndicator::ADD;
			if (!messageQueue_.try_send(static_cast<const void*>(&wrapper),
					sizeof(VectorWrapper<Object> ), priority))
			{
				APLOG_WARN << "Message queue full. Do not send.";
			}
		}
	} catch (boost::bad_any_cast&)
	{
		APLOG_ERROR << "Wrong data type of the Object to be published.";
		return;
	}

}

#endif /* UAVAP_CORE_IPC_MESSAGEQUEUEPUBLISHERIMPL_H_ */
