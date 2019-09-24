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
 * SchedulerTest.cpp
 *
 *  Created on: Jul 20, 2017
 *      Author: mircot
 */

#include <boost/test/unit_test.hpp>
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/Runner/AggregatableRunner.h"
#include "uavAP/Core/Scheduler/MicroSimulator.h"
#include "uavAP/Core/TimeProvider/SystemTimeProvider.h"

#include "uavAP/Core/Scheduler/MultiThreadingScheduler.h"

BOOST_AUTO_TEST_SUITE(SchedulerTest)

void
schedulingCount(int& counter)
{
	++counter;
}

BOOST_AUTO_TEST_CASE(MultiThreadedTest001)
{
	auto tp = std::make_shared<MicroSimulator>();
	auto sched = std::make_shared<MultiThreadingScheduler>();
	auto runner = std::make_shared<AggregatableRunner>();
	Aggregator::aggregate(
	{ tp, sched, runner });

	APLogger::instance()->setLogLevel(LogLevel::ERROR);

	int count1 = 0;

	sched->schedule(std::bind(schedulingCount, boost::ref(count1)), Milliseconds(0));

	runner->runAllStages();

	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	BOOST_CHECK_EQUAL(count1, 1);

	sched->stop();
	APLogger::instance()->setLogLevel(LogLevel::WARN);
}

BOOST_AUTO_TEST_CASE(MultiThreadedTest002)
{
	auto tp = std::make_shared<MicroSimulator>();
	auto sched = std::make_shared<MultiThreadingScheduler>();
	auto runner = std::make_shared<AggregatableRunner>();
	Aggregator::aggregate(
	{ tp, sched, runner });
	APLogger::instance()->setLogLevel(LogLevel::ERROR);

	int count1 = 0;

	tp->stopOnWait();
	auto start = tp->now();

	sched->schedule(boost::bind(schedulingCount, boost::ref(count1)), Milliseconds(0),
			Milliseconds(10));

	runner->runAllStages();

	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	BOOST_CHECK(tp->now() - start== Milliseconds(0));
	BOOST_CHECK_EQUAL(count1, 1);

	tp->releaseWait();

	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	BOOST_CHECK(tp->now() - start == Milliseconds(10));
	BOOST_CHECK_EQUAL(count1, 2);

	tp->releaseWait();

	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	BOOST_CHECK(tp->now() - start == Milliseconds(20));
	BOOST_CHECK_EQUAL(count1, 3);

	sched->stop();
	APLogger::instance()->setLogLevel(LogLevel::WARN);
}

BOOST_AUTO_TEST_CASE(MultiThreadedTest003)
{
	auto tp = std::make_shared<MicroSimulator>();
	auto sched = std::make_shared<MultiThreadingScheduler>();
	auto runner = std::make_shared<AggregatableRunner>();
	Aggregator::aggregate(
	{ tp, sched, runner });
	APLogger::instance()->setLogLevel(LogLevel::ERROR);

	int count1 = 0;
	int count2 = 0;

	auto start = tp->now();

	tp->stopOnWait();

	sched->schedule(std::bind(schedulingCount, boost::ref(count1)), Milliseconds(5),
			Milliseconds(10));

	runner->runAllStages();

	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	BOOST_CHECK_EQUAL(count1, 0);

	tp->releaseWait();
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	BOOST_CHECK(tp->now() - start== Milliseconds(5));
	BOOST_CHECK_EQUAL(count1, 1);

	sched->schedule(std::bind(schedulingCount, boost::ref(count2)), Milliseconds(0),
			Milliseconds(20));
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	BOOST_CHECK_EQUAL(count2, 1);
	tp->releaseWait();
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	BOOST_CHECK(tp->now() - start==Milliseconds(15));

	BOOST_CHECK_EQUAL(count1, 2);
	BOOST_CHECK_EQUAL(count2, 1);
	sched->stop();
	APLogger::instance()->setLogLevel(LogLevel::WARN);
}

BOOST_AUTO_TEST_CASE(MultiThreadedTest004)
{
	auto tp = std::make_shared<MicroSimulator>();
	auto sched = std::make_shared<MultiThreadingScheduler>();
	auto runner = std::make_shared<AggregatableRunner>();
	Aggregator::aggregate(
	{ tp, sched, runner });
	APLogger::instance()->setLogLevel(LogLevel::ERROR);

	int count1 = 0;

	auto start = tp->now();

	tp->stopOnWait();

	auto event = sched->schedule(std::bind(schedulingCount, std::ref(count1)), Milliseconds(5),
			Milliseconds(10));

	BOOST_REQUIRE(!runner->runAllStages());

	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	BOOST_CHECK_EQUAL(count1, 0);

	tp->releaseWait();
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	BOOST_CHECK(tp->now() - start== Milliseconds(5));
	BOOST_CHECK_EQUAL(count1, 1);

	event.cancel();
	BOOST_CHECK(event.isCancled());
	tp->releaseWait();
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	BOOST_CHECK(tp->now() - start== Milliseconds(15));
	BOOST_CHECK_EQUAL(count1, 1);

	tp->releaseWait();
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	BOOST_CHECK(tp->now() - start== Milliseconds(15));
	BOOST_CHECK_EQUAL(count1, 1);

	sched->stop();
	APLogger::instance()->setLogLevel(LogLevel::WARN);
}

BOOST_AUTO_TEST_SUITE_END()
