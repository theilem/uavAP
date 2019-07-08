/*
 * RedisChannelParams.h
 *
 *  Created on: Jan 25, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_IDC_NETWORKLAYER_REDIS_REDISCHANNELPARAMS_H_
#define UAVAP_CORE_IDC_NETWORKLAYER_REDIS_REDISCHANNELPARAMS_H_
#include <uavAP/Core/PropertyMapper/Configuration.h>
#include <string>

struct RedisChannelParams
{
	static constexpr const char* const DEFAULT_HOST_IP = "127.0.0.1";
	std::string hostIP_ = DEFAULT_HOST_IP;

	static constexpr unsigned int DEFAULT_PORT = 6379;
	unsigned int port_ = DEFAULT_PORT;

	std::string channel_;

	RedisChannelParams() = default;

	bool
	configure(const Configuration& config);
};

#endif /* UAVAP_CORE_IDC_NETWORKLAYER_REDIS_REDISCHANNELPARAMS_H_ */
