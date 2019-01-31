/*
 * RedisPublisher.cpp
 *
 *  Created on: Jan 30, 2019
 *      Author: mirco
 */

#include <uavAP/Core/IDC/NetworkLayer/Redis/RedisPublisher.h>
#include <uavAP/Core/Logging/APLogger.h>

RedisPublisher::RedisPublisher(const RedisChannelParams& params) :
		channel_(params.channel_)
{
	client_.connect(params.hostIP_, params.port_,
			[](const std::string& host, std::size_t port, cpp_redis::connect_state status)
			{	if (status == cpp_redis::connect_state::dropped)
				{	APLOG_ERROR << "Client disconnected from " << host << ": " << port;}});

}

RedisPublisher::~RedisPublisher()
{
	client_.disconnect(false);
}

bool
RedisPublisher::sendPacket(const Packet& packet)
{
	if (!client_.is_connected())
	{
		APLOG_ERROR << "Client is not connected, cannot send.";
		return false;
	}
	client_.publish(channel_, packet.getBuffer());
	client_.commit();
	return true;
}

void
RedisPublisher::startHandler()
{
}
