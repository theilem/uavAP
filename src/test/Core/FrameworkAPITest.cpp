/*
 * FrameworkAPITest.cpp
 *
 *  Created on: Feb 4, 2019
 *      Author: mirco
 */

#include <boost/test/unit_test.hpp>

#include <uavAP/Core/Framework/api/FrameworkAPI.h>
#include <uavAP/Core/Framework/Helper.h>
#include <uavAP/Core/IDC/NetworkLayer/NetworkFactory.h>

namespace
{
class TestHelper: public Helper
{
public:
	TestHelper()
	{
		addFactory<NetworkFactory>();
	}
};
}

BOOST_AUTO_TEST_SUITE(FrameworkAPITest)

BOOST_AUTO_TEST_CASE(locking)
{
	auto lock = FrameworkAPI::lockAggregator();
	lock.unlock();
	auto lock2 = FrameworkAPI::lockAggregator();
}

BOOST_AUTO_TEST_CASE(aggregation_merge)
{
	TestHelper helper;
	{
		Aggregator agg = helper.createAggregation("Core/config/agg1.json");
		auto lock = FrameworkAPI::lockAggregator();
		auto globalAgg = FrameworkAPI::getAggregator();
		globalAgg->merge(agg);
	}

	{
		auto lock = FrameworkAPI::lockAggregator();
		auto globalAgg = FrameworkAPI::getAggregator();
		auto net1 = globalAgg->getOne<INetworkLayer>();
		BOOST_CHECK(net1);
	}

	{
		Aggregator agg = helper.createAggregation("Core/config/agg2.json");
		auto lock = FrameworkAPI::lockAggregator();
		auto globalAgg = FrameworkAPI::getAggregator();
		globalAgg->merge(agg);
	}

	{
		auto lock = FrameworkAPI::lockAggregator();
		auto globalAgg = FrameworkAPI::getAggregator();
		auto nets = globalAgg->getAll<INetworkLayer>();
		BOOST_CHECK_EQUAL(nets.size(), 2);
	}

	{
		auto lock = FrameworkAPI::lockAggregator();
		auto globalAgg = FrameworkAPI::getAggregator();
		auto netSerials = globalAgg->getAll<SerialNetworkLayer>();
		BOOST_CHECK_EQUAL(netSerials.size(), 1);
		auto netRedis = globalAgg->getAll<RedisNetworkLayer>();
		BOOST_CHECK_EQUAL(netRedis.size(), 1);

		BOOST_CHECK(netSerials.front());
		BOOST_CHECK(netRedis.front());
	}

	{
		auto lock = FrameworkAPI::lockAggregator();
		auto globalAgg = FrameworkAPI::getAggregator();
		globalAgg->clear();
	}

	{
		auto lock = FrameworkAPI::lockAggregator();
		auto globalAgg = FrameworkAPI::getAggregator();
		auto netSerials = globalAgg->getAll<SerialNetworkLayer>();
		BOOST_CHECK_EQUAL(netSerials.size(), 0);
		auto netRedis = globalAgg->getAll<RedisNetworkLayer>();
		BOOST_CHECK_EQUAL(netRedis.size(), 0);
		auto nets = globalAgg->getAll<INetworkLayer>();
		BOOST_CHECK_EQUAL(nets.size(), 0);
	}

}

BOOST_AUTO_TEST_SUITE_END()

