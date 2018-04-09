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

#include <boost/bind/bind.hpp>
#include <boost/test/unit_test.hpp>
#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/Core/Runner/SimpleRunner.h"
#include "uavAP/Core/Scheduler/MultiThreadingScheduler.h"
#include "uavAP/Core/Time.h"
#include "uavAP/Core/TimeProvider/SystemTimeProvider.h"

#include "uavAP/Core/IPC/IPC.h"

BOOST_AUTO_TEST_SUITE(IPCTest)

struct Test
{
	int val1;
	double val2;
	Vector3 val3;
} test;

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
	auto agg = Aggregator::aggregate({ipc, tp, sched});

	Publisher publisher = ipc->publishOnSharedMemory<Test>("test");

	int counter1 = 0;
	int counter2 = 0;
	Subscription sub1 = ipc->subscribeOnSharedMemory<Test>("test", boost::bind(&checkValue, _1, boost::ref(counter1)));
	Subscription sub2 = ipc->subscribeOnSharedMemory<Test>("test", boost::bind(&checkValue, _1, boost::ref(counter2)));
	BOOST_REQUIRE(sub1.connected());
	BOOST_REQUIRE(sub2.connected());
	SimpleRunner run(agg);
	BOOST_REQUIRE(!run.runAllStages());
	test.val1 = 1;
	test.val2 = 1.2;
	test.val3 = Vector3(1, 2, 3);

	BOOST_CHECK_EQUAL(counter1, 0);
	BOOST_CHECK_EQUAL(counter2, 0);

	boost::this_thread::sleep(Milliseconds(5));

	BOOST_CHECK_EQUAL(counter1, 0);
	BOOST_CHECK_EQUAL(counter2, 0);
	publisher.publish(test);
	boost::this_thread::sleep(Milliseconds(5));

	BOOST_CHECK_EQUAL(counter1, 1);
	BOOST_CHECK_EQUAL(counter2, 1);

	test.val1 = 2;
	test.val2 = 2.2;
	test.val3 = Vector3(2, 3, 4);

	publisher.publish(test);
	boost::this_thread::sleep(Milliseconds(5));

	BOOST_CHECK_EQUAL(counter1, 2);
	BOOST_CHECK_EQUAL(counter2, 2);
}

BOOST_AUTO_TEST_CASE(test002)
{
	auto ipc = std::make_shared<IPC>();
	auto tp = std::make_shared<SystemTimeProvider>();
	auto sched = std::make_shared<MultiThreadingScheduler>();
	auto agg = Aggregator::aggregate({ipc, tp, sched});

	Publisher publisher = ipc->publishOnMessageQueue<Test>("test2", 5);
	int counter1 = 0;
	Subscription sub = ipc->subscribeOnMessageQueue<Test>("test2", boost::bind(&checkValue, _1, boost::ref(counter1)));
	BOOST_REQUIRE(sub.connected());
	SimpleRunner run(agg);
	BOOST_REQUIRE(!run.runAllStages());
	test.val1 = 1;
	test.val2 = 1.2;
	test.val3 = Vector3(1, 2, 3);
	boost::this_thread::sleep(Milliseconds(10));
	publisher.publish(test);
	publisher.publish(test);
	publisher.publish(test);
	publisher.publish(test);
	publisher.publish(test);
	publisher.publish(test);
	boost::this_thread::sleep(Milliseconds(10));

	BOOST_CHECK_EQUAL(counter1, 6);

	test.val1 = 2;
	test.val2 = 2.2;
	test.val3 = Vector3(2, 3, 4);

	publisher.publish(test);
	boost::this_thread::sleep(Milliseconds(10));

	BOOST_CHECK_EQUAL(counter1, 7);
}

BOOST_AUTO_TEST_SUITE_END()

