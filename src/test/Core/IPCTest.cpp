////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/*
 * IPCTest.cpp
 *
 *  Created on: Jul 18, 2017
 *      Author: mircot
 */

#include <boost/test/unit_test.hpp>
#include <uavAP/Core/SensorData.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>
#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/Core/Runner/SimpleRunner.h"
#include "uavAP/Core/Scheduler/MultiThreadingScheduler.h"
#include "uavAP/Core/Time.h"
#include "uavAP/Core/TimeProvider/SystemTimeProvider.h"

struct Test
{
	int val1;
	double val2;
	Vector3 val3;
} test;

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, Test& t)
{
	ar & t.val1;
	ar & t.val2;
	ar & t.val3;
}
}

#include "uavAP/Core/IPC/IPC.h"
#include <functional>

BOOST_AUTO_TEST_SUITE(IPCTest)


std::vector<Test> testVec;

void
checkValue(const Test& t, int& counter)
{
	BOOST_CHECK_EQUAL(t.val1, test.val1);
	BOOST_CHECK_EQUAL(t.val2, test.val2);
	BOOST_CHECK_EQUAL(t.val3, test.val3);
	++counter;
}

void
checkSensorData(const SensorData& t, int& counter)
{
	BOOST_CHECK_EQUAL(t.position.x(), 1);
	BOOST_CHECK_EQUAL(t.position.y(), 2);
	BOOST_CHECK_EQUAL(t.position.z(), 3);
	++counter;
}

void
checkControllerOutput(const ControllerOutput& t, int& counter)
{
	BOOST_CHECK_EQUAL(t.pitchOutput, 1);
	BOOST_CHECK_EQUAL(t.throttleOutput, 2);
	BOOST_CHECK_EQUAL(t.yawOutput, 3);
	++counter;
}

void
checkVector(const std::vector<Test>& t, int& counter)
{
	BOOST_REQUIRE_EQUAL(t.size(), testVec.size());
	int k = 0;
	for (auto& it : testVec)
	{
		BOOST_CHECK_EQUAL(t[k].val1, it.val1);
		BOOST_CHECK_EQUAL(t[k].val2, it.val2);
		BOOST_CHECK_EQUAL(t[k].val3, it.val3);
		k++;
	}
	++counter;
}

BOOST_AUTO_TEST_CASE(test001)
{
	auto ipc = std::make_shared<IPC>();
	auto tp = std::make_shared<SystemTimeProvider>();
	auto sched = std::make_shared<MultiThreadingScheduler>();
	auto agg = Aggregator::aggregate(
	{ ipc, tp, sched });

	auto publisher = ipc->publish<Test>("test");

	int counter1 = 0;
	int counter2 = 0;
	Subscription sub1 = ipc->subscribe<Test>("test",
			boost::bind(&checkValue, _1, boost::ref(counter1)));
	Subscription sub2 = ipc->subscribe<Test>("test",
			boost::bind(&checkValue, _1, boost::ref(counter2)));
	BOOST_REQUIRE(sub1.connected());
	BOOST_REQUIRE(sub2.connected());
	SimpleRunner run(agg);
	BOOST_REQUIRE(!run.runAllStages());
	test.val1 = 1;
	test.val2 = 1.2;
	test.val3 = Vector3(1, 2, 3);

	BOOST_CHECK_EQUAL(counter1, 0);
	BOOST_CHECK_EQUAL(counter2, 0);

	std::this_thread::sleep_for(std::chrono::milliseconds(5));

	BOOST_CHECK_EQUAL(counter1, 0);
	BOOST_CHECK_EQUAL(counter2, 0);
	publisher.publish(test);
	std::this_thread::sleep_for(std::chrono::milliseconds(5));

	BOOST_CHECK_EQUAL(counter1, 1);
	BOOST_CHECK_EQUAL(counter2, 1);

	test.val1 = 2;
	test.val2 = 2.2;
	test.val3 = Vector3(2, 3, 4);

	publisher.publish(test);
	std::this_thread::sleep_for(std::chrono::milliseconds(5));

	BOOST_CHECK_EQUAL(counter1, 2);
	BOOST_CHECK_EQUAL(counter2, 2);
}

BOOST_AUTO_TEST_CASE(test002)
{
	auto ipc = std::make_shared<IPC>();
	auto tp = std::make_shared<SystemTimeProvider>();
	auto sched = std::make_shared<MultiThreadingScheduler>();
	auto dp = std::make_shared<DataPresentation>();
	auto agg = Aggregator::aggregate(
	{ ipc, tp, sched, dp });

	IPCOptions opt;
	opt.multiTarget = true; // setting to shared memory
	auto publisher = ipc->publish<SensorData>("test3", opt);
	int counter1 = 0;
	Subscription sub = ipc->subscribe<SensorData>("test3",
			std::bind(&checkSensorData, std::placeholders::_1, std::ref(counter1)));
	BOOST_REQUIRE(sub.connected());
	SimpleRunner run(agg);
	BOOST_REQUIRE(!run.runAllStages());
	SensorData data;
	data.position = Vector3(1,2,3);

	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	publisher.publish(data);
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	publisher.publish(data);
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	BOOST_CHECK_EQUAL(counter1, 2);

}

BOOST_AUTO_TEST_CASE(test003)
{
	auto ipc = std::make_shared<IPC>();
	auto tp = std::make_shared<SystemTimeProvider>();
	auto sched = std::make_shared<MultiThreadingScheduler>();
	auto dp = std::make_shared<DataPresentation>();
	auto agg = Aggregator::aggregate(
	{ ipc, tp, sched, dp });

	IPCOptions opt;
	opt.multiTarget = true; // setting to shared memory
	auto publisher = ipc->publish<ControllerOutput>("test55", opt);
	int counter1 = 0;
	Subscription sub = ipc->subscribe<ControllerOutput>("test55",
			std::bind(&checkControllerOutput, std::placeholders::_1, std::ref(counter1)));
	BOOST_REQUIRE(sub.connected());
	SimpleRunner run(agg);
	BOOST_REQUIRE(!run.runAllStages());
	ControllerOutput data;
	data.pitchOutput = 1;
	data.throttleOutput = 2;
	data.yawOutput = 3;

	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	publisher.publish(data);
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	publisher.publish(data);
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	BOOST_CHECK_EQUAL(counter1, 2);

}

BOOST_AUTO_TEST_CASE(test004)
{
	auto ipc = std::make_shared<IPC>();
	auto tp = std::make_shared<SystemTimeProvider>();
	auto sched = std::make_shared<MultiThreadingScheduler>();
	auto agg = Aggregator::aggregate(
	{ ipc, tp, sched });

	IPCOptions opt;
	opt.multiTarget = false; // setting to message queue
	auto publisher = ipc->publish<Test>("test2", opt);
	int counter1 = 0;
	Subscription sub = ipc->subscribe<Test>("test2",
			std::bind(&checkValue, std::placeholders::_1, std::ref(counter1)), opt);
	BOOST_REQUIRE(sub.connected());
	SimpleRunner run(agg);
	BOOST_REQUIRE(!run.runAllStages());
	test.val1 = 1;
	test.val2 = 1.2;
	test.val3 = Vector3(1, 2, 3);
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	publisher.publish(test);
	publisher.publish(test);
	publisher.publish(test);
	publisher.publish(test);
	publisher.publish(test);
	publisher.publish(test);
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	BOOST_CHECK_EQUAL(counter1, 6);

	test.val1 = 2;
	test.val2 = 2.2;
	test.val3 = Vector3(2, 3, 4);

	publisher.publish(test);
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	BOOST_CHECK_EQUAL(counter1, 7);
}

BOOST_AUTO_TEST_SUITE_END()
