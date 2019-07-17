/*
 * MessageQueuePublisherImpl.cpp
 *
 *  Created on: Jul 15, 2019
 *      Author: mirco
 */
#include <uavAP/Core/IPC/detail/MessageQueuePublisherImpl.h>
#include <uavAP/Core/Logging/APLogger.h>

MessageQueuePublisherImpl::MessageQueuePublisherImpl(const std::string& id, unsigned int numOfMessages,
		std::size_t elementSize) :
		messageQueue_(boost::interprocess::create_only, id.c_str(), numOfMessages, elementSize), id_(
				id), maxPacketSize_(elementSize)
{
}

MessageQueuePublisherImpl::~MessageQueuePublisherImpl()
{
	if (messageQueue_.remove(id_.c_str()))
		APLOG_DEBUG << id_ << " message queue removed.";
}

void
MessageQueuePublisherImpl::publish(const Packet& packet)
{
	if (packet.getSize() > maxPacketSize_)
	{
		APLOG_ERROR << "Packet to large: " << packet.getSize() << " > " << maxPacketSize_;
		return;
	}
	unsigned int priority = 1; //TODO Adjust priority?
	if (!messageQueue_.try_send(static_cast<const void*>(packet.getStart()), packet.getSize(), priority))
	{
		APLOG_WARN << "Message queue full. Do not send.";
	}
}
