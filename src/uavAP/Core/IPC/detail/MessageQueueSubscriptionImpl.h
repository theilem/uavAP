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
#include "uavAP/Core/IPC/detail/ISubscriptionImpl.h"
#include <thread>

class Packet;

class MessageQueueSubscriptionImpl: public ISubscriptionImpl
{
public:

	MessageQueueSubscriptionImpl(const std::string& id, std::size_t maxPacketSize);

	~MessageQueueSubscriptionImpl();

	void
	cancel() override;

	void
	start() override;

	boost::signals2::connection
	subscribe(const OnPacketSlot& slot) override;

private:

	void
	onMessageQueue();

	boost::interprocess::message_queue messageQueue_;

	OnPacket onMessageQueue_;

	std::thread listenerThread_;
	std::atomic_bool listenerCanceled_;

	std::size_t maxPacketSize_;

	std::string id_;
};

#endif /* UAVAP_CORE_IPC_DETAIL_MESSAGEQUEUESUBSCRIPTIONIMPL_H_ */
