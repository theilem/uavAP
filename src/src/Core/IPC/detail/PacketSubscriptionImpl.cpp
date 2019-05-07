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
 * PacketSubscriptionImpl.cpp
 *
 *  Created on: Aug 9, 2017
 *      Author: mircot
 */
#include "uavAP/Core/IPC/detail/PacketSubscriptionImpl.h"

PacketSubscriptionImpl::PacketSubscriptionImpl(const std::string& id, std::size_t maxPacketSize) :
		messageQueue_(boost::interprocess::open_only, id.c_str()), listenerCanceled_(false), maxPacketSize_(
				maxPacketSize), id_(id)
{
}

PacketSubscriptionImpl::~PacketSubscriptionImpl()
{
	if (!listenerCanceled_.load())
	{
		cancel();
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); //Wait for timeout of message_queue to stop subscription

//	if (messageQueue_.remove(id_.c_str()))
//		APLOG_DEBUG << id_ << " message queue removed.";
}

void
PacketSubscriptionImpl::cancel()
{
	listenerCanceled_.store(true);
}

void
PacketSubscriptionImpl::start()
{
	listenerThread_ = std::thread(std::bind(&PacketSubscriptionImpl::onMessageQueue, this));
	listenerThread_.detach();
}

boost::signals2::connection
PacketSubscriptionImpl::subscribe(const OnMessageQueueSlot& slot)
{
	return onMessageQueue_.connect(slot);
}

void
PacketSubscriptionImpl::onMessageQueue()
{
	using namespace boost::interprocess;

	for (;;)
	{
		if (listenerCanceled_.load())
		{
			return;
		}
		std::string packet;
		packet.resize(maxPacketSize_);
		message_queue::size_type size;
		unsigned int priority;
		auto timeout = boost::get_system_time() + Milliseconds(100);
		if (!messageQueue_.timed_receive(static_cast<void*>(&packet[0]), maxPacketSize_, size,
				priority, timeout))
		{
			continue;
		}
		packet.resize(size);
		packet += ' ';
		onMessageQueue_(Packet(packet));
	}
}
