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
 * PacketPublisherImpl.h
 *
 *  Created on: Aug 9, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_IPC_DETAIL_PACKETPUBLISHERIMPL_H_
#define UAVAP_CORE_IPC_DETAIL_PACKETPUBLISHERIMPL_H_
#include <boost/interprocess/ipc/message_queue.hpp>
#include "uavAP/Core/IPC/detail/IPublisherImpl.h"


class PacketPublisherImpl: public IPublisherImpl
{
public:

	PacketPublisherImpl(const std::string& id, unsigned int numOfMessages, std::size_t elementSize);

	~PacketPublisherImpl();

	void
	publish(const boost::any& obj) override;

	void
	publish(const std::vector<boost::any>& vec) override;

private:

	boost::interprocess::message_queue messageQueue_;

	std::string id_;

	std::size_t maxPacketSize_;

};



#endif /* UAVAP_CORE_IPC_DETAIL_PACKETPUBLISHERIMPL_H_ */
