/*
 * SharedMemoryPublisherImpl.cpp
 *
 *  Created on: Jul 15, 2019
 *      Author: mirco
 */
#include <uavAP/Core/IPC/detail/SharedMemoryPublisherImpl.h>

SharedMemoryPublisherImpl::SharedMemoryPublisherImpl(const std::string& id, std::size_t init) :
		maxPacketSize_(init)
{
	using namespace boost::interprocess;
	try
	{
		sharedMem_ = shared_memory_object(open_only, id.c_str(), read_write);
		APLOG_WARN << "Shared memory object " << id << " aready exists. Using existing.";
	} catch (interprocess_exception&)
	{
		sharedMem_ = shared_memory_object(create_only, id.c_str(), read_write);
		sharedMem_.truncate(sizeof(MessageObjectHeader) + maxPacketSize_);
	}

	mapped_region region(sharedMem_, read_write);
	MessageObjectHeader header;
	header.maxPacketSize = maxPacketSize_;

	memcpy(region.get_address(), &header, sizeof(header));

}

void
SharedMemoryPublisherImpl::publish(const Packet& packet)
{
	using namespace boost::interprocess;

	mapped_region region(sharedMem_, read_write);
	MessageObjectHeader* message = static_cast<MessageObjectHeader*>(region.get_address());

	if (packet.getSize() > maxPacketSize_)
	{
		APLOG_ERROR << "Packet to big, cannot copy to shm.";
		return;
	}

	boost::unique_lock<boost::interprocess::interprocess_sharable_mutex> lock(message->mtx);

	memcpy(region.get_address() + sizeof(MessageObjectHeader), packet.getStart(), packet.getSize());

	message->cnd.notify_all();
}

SharedMemoryPublisherImpl::~SharedMemoryPublisherImpl()
{
	std::string name = sharedMem_.get_name();
	if (sharedMem_.remove(name.c_str()))
		APLOG_DEBUG << name << " shared memory removed.";
}

