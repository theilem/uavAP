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
 * PacketPublisherImpl.cpp
 *
 *  Created on: Aug 9, 2017
 *      Author: mircot
 */
#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/Core/IPC/detail/PacketPublisherImpl.h"
#include "uavAP/Core/Logging/APLogger.h"

PacketPublisherImpl::PacketPublisherImpl(const std::string& id, unsigned int numOfMessages,
		std::size_t elementSize) :
		messageQueue_(boost::interprocess::create_only, id.c_str(), numOfMessages, elementSize), id_(
				id), maxPacketSize_(elementSize)
{
}

PacketPublisherImpl::~PacketPublisherImpl()
{
	if (messageQueue_.remove(id_.c_str()))
		APLOG_DEBUG << id_ << " message queue removed.";
}

void
PacketPublisherImpl::publish(const boost::any& obj)
{
	Packet packet;
	try
	{
		packet = boost::any_cast<Packet>(obj);
	} catch (boost::bad_any_cast&)
	{
		APLOG_ERROR << "Not a packet.";
		return;
	}
	if (packet.getSize() > maxPacketSize_)
	{
		APLOG_ERROR << "Packet to large: " << packet.getSize() << " > " << maxPacketSize_;
		return;
	}
	unsigned int priority = 1; //TODO Adjust priority?
	if (!messageQueue_.try_send(packet.getStart(), packet.getSize(), priority))
	{
		APLOG_WARN << "Message queue full. Do not send.";
	}
}

void
PacketPublisherImpl::publish(const std::vector<boost::any>& vec)
{
	APLOG_ERROR << "Vector Packets not implemented.";
}
