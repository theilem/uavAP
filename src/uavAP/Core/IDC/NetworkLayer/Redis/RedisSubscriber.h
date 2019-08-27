/*
 * RedisSubscriber.h
 *
 *  Created on: Jan 30, 2019
 *      Author: mirco
 */

#ifndef PLUGIN_NETWORKLAYER_REDIS_REDISSUBSCRIBER_H_
#define PLUGIN_NETWORKLAYER_REDIS_REDISSUBSCRIBER_H_



#include <boost/signals2.hpp>
#include <uavAP/Core/DataPresentation/Packet.h>
#include <uavAP/Core/IDC/NetworkLayer/Redis/RedisChannelParams.h>

#include <cpp_redis/cpp_redis>
#include <tacopie/tacopie>


class RedisSubscriber
{
public:

	RedisSubscriber(const RedisChannelParams& params);

	~RedisSubscriber();

	using OnPacket = boost::signals2::signal<void(const Packet&)>;

	boost::signals2::connection
	subscribeOnPackets(const OnPacket::slot_type& slot);

	void
	startHandler();
private:

	void
	onChannel(const std::string& channel, const std::string& message);

	cpp_redis::subscriber subscriber_;

	OnPacket onPacket_;

};



#endif /* PLUGIN_NETWORKLAYER_REDIS_REDISSUBSCRIBER_H_ */
