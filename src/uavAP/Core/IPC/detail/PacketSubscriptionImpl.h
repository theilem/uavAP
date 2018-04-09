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
 * PacketSubscriptionImpl.h
 *
 *  Created on: Aug 9, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_IPC_DETAIL_PACKETSUBSCRIPTIONIMPL_H_
#define UAVAP_CORE_IPC_DETAIL_PACKETSUBSCRIPTIONIMPL_H_

#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/signals2.hpp>
#include <boost/thread/thread_time.hpp>
#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/Core/IPC/detail/ISubscriptionImpl.h"
#include "uavAP/Core/IPC/detail/MessageObject.h"
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/Time.h"
#include <thread>

class PacketSubscriptionImpl: public ISubscriptionImpl
{
public:

	PacketSubscriptionImpl(const std::string& id, std::size_t maxPacketSize);

	~PacketSubscriptionImpl();

	void
	cancel() override;

	void
	start() override;

	using OnMessageQueue = boost::signals2::signal<void(const Packet&)>;
	using OnMessageQueueSlot = boost::function<void(const Packet&)>;

	boost::signals2::connection
	subscribe(const OnMessageQueueSlot& slot);

private:

	void
	onMessageQueue();

	boost::interprocess::message_queue messageQueue_;

	OnMessageQueue onMessageQueue_;

	std::thread listenerThread_;
	std::atomic_bool listenerCanceled_;

	std::size_t maxPacketSize_;

	std::string id_;
};

#endif /* UAVAP_CORE_IPC_DETAIL_PACKETSUBSCRIPTIONIMPL_H_ */
