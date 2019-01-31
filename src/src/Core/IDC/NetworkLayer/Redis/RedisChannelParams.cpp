/*
 * RedisChannelParams.cpp
 *
 *  Created on: Jan 25, 2019
 *      Author: mirco
 */
#include <uavAP/Core/IDC/NetworkLayer/Redis/RedisChannelParams.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>

bool
RedisChannelParams::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	pm.add("host_ip", hostIP_, false);
	pm.add<unsigned int>("port", port_, false);
	pm.add("channel", channel_, true);

	return pm.map();
}
