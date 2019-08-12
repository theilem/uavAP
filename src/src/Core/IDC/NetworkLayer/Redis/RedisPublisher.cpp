/*
 * RedisPublisher.cpp
 *
 *  Created on: Jan 30, 2019
 *      Author: mirco
 */

#include <uavAP/Core/IDC/NetworkLayer/Redis/RedisPublisher.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/Time.h>

RedisPublisher::RedisPublisher(const RedisChannelParams& params) :
		channel_(params.channel_), auth_(params.auth_)
{
	client_.connect(params.hostIP_, params.port_,
			std::bind(&RedisPublisher::onConnectionEvent, this, std::placeholders::_1,
					std::placeholders::_2, std::placeholders::_3));
//	if (!params.auth_.empty())
//	{
//		APLOG_DEBUG << "Authenticating with " << params.auth_;
//		auto future = client_.auth(params.auth_);
//
//		APLOG_DEBUG << "Wait for reply";
//		auto status = future.wait_for(Seconds(1));
//		if (status == std::future_status::timeout)
//		{
//			APLOG_ERROR << "Authentication timed out";
//			return;
//		}
//		if (status == std::future_status::deferred)
//		{
//			APLOG_ERROR << "Authentication deferred";
//			return;
//		}
//		auto reply = future.get();
//		APLOG_DEBUG << "Received reply";
////		future.wait_for(Seconds(1));
//		if (reply.is_error())
//		{
//			APLOG_ERROR << reply.error();
//		}
//	}

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

void
RedisPublisher::onConnectionEvent(const std::string& host, std::size_t port,
		cpp_redis::connect_state status)
{
	APLOG_DEBUG << "Connection event " << static_cast<int>(status);
	if (status == cpp_redis::connect_state::ok)
	{
		if (auth_.empty())
		{
			APLOG_DEBUG << "Connection started";
			return;
		}

		APLOG_DEBUG << "Authenticating with " << auth_;
		auto future = client_.auth(auth_);
		client_.commit();

		auto status = future.wait_for(Seconds(1));
		if (status == std::future_status::timeout)
		{
			APLOG_ERROR << "Authentication timed out";
			return;
		}
		if (status == std::future_status::deferred)
		{
			APLOG_ERROR << "Authentication deferred";
			return;
		}
		auto reply = future.get();
		if (reply.is_error())
		{
			APLOG_ERROR << reply.error();
		}
	}

	if (status == cpp_redis::connect_state::dropped)
	{
		APLOG_ERROR << "Client disconnected from " << host << ": " << port;
	}
}
