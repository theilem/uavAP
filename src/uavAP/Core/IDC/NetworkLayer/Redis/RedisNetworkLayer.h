/*
 * RedisNetworkLayer.h
 *
 *  Created on: Jan 25, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_IDC_NETWORKLAYER_REDIS_REDISNETWORKLAYER_H_
#define UAVAP_CORE_IDC_NETWORKLAYER_REDIS_REDISNETWORKLAYER_H_
#include <uavAP/Core/IDC/NetworkLayer/INetworkLayer.h>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Runner/IRunnableObject.h>
#include "uavAP/Core/PropertyMapper/Configuration.h"
#include <unordered_map>

class IScheduler;
class RedisPublisher;
class RedisSubscriber;

class RedisNetworkLayer: public INetworkLayer, public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "redis";

	static std::shared_ptr<RedisNetworkLayer>
	create(const Configuration& config);

	bool
	configure(const Configuration& config);

	bool
	sendPacket(const std::string& id, const Packet& packet) override;

	boost::signals2::connection
	subscribeOnPacket(const std::string& id, const OnPacket::slot_type& handle) override;

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

private:

	std::unordered_map<std::string, std::shared_ptr<RedisSubscriber>> subscribers_;
	std::unordered_map<std::string, std::shared_ptr<RedisPublisher>> publishers_;

	ObjectHandle<IScheduler> scheduler_;

};

#endif /* UAVAP_CORE_IDC_NETWORKLAYER_REDIS_REDISNETWORKLAYER_H_ */
