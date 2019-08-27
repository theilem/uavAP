/*
 * RedisPublisher.h
 *
 *  Created on: Jan 30, 2019
 *      Author: mirco
 */

#ifndef PLUGIN_NETWORKLAYER_REDIS_REDISPUBLISHER_H_
#define PLUGIN_NETWORKLAYER_REDIS_REDISPUBLISHER_H_




#include <boost/signals2.hpp>
#include <uavAP/Core/DataPresentation/Packet.h>
#include <uavAP/Core/IDC/NetworkLayer/Redis/RedisChannelParams.h>

#include <cpp_redis/cpp_redis>
#include <tacopie/tacopie>


class RedisPublisher
{
public:

	RedisPublisher(const RedisChannelParams& params);

	~RedisPublisher();

	using OnPacket = boost::signals2::signal<void(const Packet&)>;

	bool
	sendPacket(const Packet& packet);

	void
	startHandler();
private:

	void
	onConnectionEvent(const std::string& host, std::size_t port, cpp_redis::connect_state status);

	cpp_redis::client client_;
	std::string channel_;
	std::string auth_;

};


#endif /* PLUGIN_NETWORKLAYER_REDIS_REDISPUBLISHER_H_ */
