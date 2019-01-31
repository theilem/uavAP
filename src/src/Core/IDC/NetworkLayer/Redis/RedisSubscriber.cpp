/*
 * RedisSubscriber.cpp
 *
 *  Created on: Jan 30, 2019
 *      Author: mirco
 */

#include <uavAP/Core/IDC/NetworkLayer/Redis/RedisSubscriber.h>
#include <uavAP/Core/Logging/APLogger.h>

RedisSubscriber::RedisSubscriber(const RedisChannelParams& params)
{
	subscriber_.connect(params.hostIP_, params.port_,
			[](const std::string& host, std::size_t port, cpp_redis::connect_state status)
			{	if (status == cpp_redis::connect_state::dropped)
				{	APLOG_ERROR << "Client disconnected from " << host << ": " << port;}});
	subscriber_.subscribe(params.channel_,
			std::bind(&RedisSubscriber::onChannel, this, std::placeholders::_1,
					std::placeholders::_2));
}

RedisSubscriber::~RedisSubscriber()
{
	subscriber_.disconnect(false);
}

boost::signals2::connection
RedisSubscriber::subscribeOnPackets(const OnPacket::slot_type& slot)
{
	return onPacket_.connect(slot);
}

void
RedisSubscriber::startHandler()
{
	subscriber_.commit();
}

void
RedisSubscriber::onChannel(const std::string& channel, const std::string& message)
{
	onPacket_(Packet(message));
}
