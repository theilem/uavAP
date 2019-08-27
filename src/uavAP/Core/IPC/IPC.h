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
 * IPC.h
 *
 *  Created on: Jul 18, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_IPC_IPC_H_
#define UAVAP_CORE_IPC_IPC_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/sharable_lock.hpp>
#include <uavAP/Core/DataPresentation/DataPresentation.h>
#include <uavAP/Core/DataPresentation/BinarySerialization.hpp>
#include <uavAP/Core/IPC/IPCOptions.h>
#include <uavAP/Core/IPC/IPCParams.h>
#include <uavAP/Core/Object/AggregatableObject.hpp>
#include <uavAP/Core/Object/AggregatableObjectImpl.hpp>
#include <uavAP/Core/PropertyMapper/ConfigurableObject.hpp>

#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/Core/IPC/detail/ISubscriptionImpl.h"
#include "uavAP/Core/IPC/detail/MessageQueuePublisherImpl.h"
#include "uavAP/Core/IPC/detail/MessageQueueSubscriptionImpl.h"
#include "uavAP/Core/IPC/detail/SharedMemoryPublisherImpl.h"
#include "uavAP/Core/IPC/detail/SharedMemorySubscriptionImpl.h"
#include "uavAP/Core/IPC/Publisher.h"
#include "uavAP/Core/IPC/Subscription.h"
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/Scheduler/IScheduler.h"

class SignalHandler;

class IPC: public IRunnableObject, public AggregatableObject<IScheduler, DataPresentation,
		SignalHandler>, public ConfigurableObject<IPCParams>
{

public:

	static constexpr TypeId typeId = "ipc";

	IPC();

	~IPC();

	static std::shared_ptr<IPC>
	create(const Configuration& config);

	template<typename Type>
	Subscription
	subscribe(const std::string& id, const std::function<void
	(const Type&)>& slot, const IPCOptions& options = IPCOptions());

	Subscription
	subscribeOnPackets(const std::string& id, const std::function<void
	(const Packet&)>& slot, const IPCOptions& options = IPCOptions());

	template<typename Type>
	Publisher<Type>
	publish(const std::string& id, const IPCOptions& options = IPCOptions());

	Publisher<Packet>
	publishPackets(const std::string& id, const IPCOptions& options = IPCOptions());

	bool
	run(RunStage stage) override;

private:

	Subscription
	subscribeOnSharedMemory(const std::string& id, const std::function<void
	(const Packet&)>& slot);

	Subscription
	subscribeOnMessageQueue(const std::string& id, const std::function<void
	(const Packet&)>& slot);

	std::shared_ptr<IPublisherImpl>
	publishOnSharedMemory(const std::string& id, unsigned maxPacketSize);

	std::shared_ptr<IPublisherImpl>
	publishOnMessageQueue(const std::string& id, unsigned maxPacketSize, unsigned numPackets);

	void
	sigintHandler(int sig);

	template<typename Type>
	void
	forwardFromPacket(const Packet& packet, const std::function<void
	(const Type&)>& func) const;

	template<typename Type>
	Packet
	forwardToPacket(const Type& val) const;

	std::vector<std::shared_ptr<IPublisherImpl>> publications_;

	std::mutex subscribeMutex_;
	std::map<std::string, std::shared_ptr<ISubscriptionImpl>> subscriptions_;

};


template<typename Type>
inline Publisher<Type>
IPC::publish(const std::string& id, const IPCOptions& options)
{
	auto packetSize = forwardToPacket(Type()).getSize();
	auto forwarding = std::bind(&IPC::forwardToPacket<Type>, this, std::placeholders::_1);
	if (options.multiTarget)
	{
		return Publisher<Type>(publishOnSharedMemory(id, packetSize), forwarding);
	}
	else
	{
		return Publisher<Type>(publishOnMessageQueue(id, packetSize, params.maxNumPackets()), forwarding);
	}
}

template<typename Type>
inline Subscription
IPC::subscribe(const std::string& id, const std::function<void
(const Type&)>& slot, const IPCOptions& options)
{
	auto packetSlot = std::bind(&IPC::forwardFromPacket<Type>, this, std::placeholders::_1, slot);
	if (options.multiTarget)
	{
		return subscribeOnSharedMemory(id, packetSlot);
	}
	else
	{
		return subscribeOnMessageQueue(id, packetSlot);
	}
}

template<typename Type>
inline void
IPC::forwardFromPacket(const Packet& packet, const std::function<void
(const Type&)>& func) const
{

	if (auto dp = get<DataPresentation>())
		func(dp->deserialize<Type>(packet));
	else
		func(dp::deserialize<Type>(packet));

}

template<typename Type>
inline Packet
IPC::forwardToPacket(const Type& val) const
{
	if (auto dp = get<DataPresentation>())
		return dp->serialize(val);
	else
		return dp::serialize(val);

}


#endif /* UAVAP_CORE_IPC_IPC_H_ */
