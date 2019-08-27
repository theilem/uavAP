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
#include "uavAP/Core/IPC/detail/ISubscriptionImpl.h"
#include "uavAP/Core/IPC/detail/MessageObject.h"
#include "uavAP/Core/Time.h"
#include <thread>

class Packet;

class SharedMemorySubscriptionImpl: public ISubscriptionImpl
{
public:

	SharedMemorySubscriptionImpl(const std::string& id);

	~SharedMemorySubscriptionImpl();

	void
	cancel() override;

	void
	start() override;

	boost::signals2::connection
	subscribe(const OnPacketSlot& slot) override;

private:

	void
	onSharedMemory();

	boost::interprocess::shared_memory_object sharedMem_;

	OnPacket onSharedMem_;

	std::thread listenerThread_;

	std::atomic_bool listenerCanceled_;
};
#endif /* UAVAP_CORE_IPC_DETAIL_SHAREDMEMORYSUBSCRIPTIONIMPL_H_ */
