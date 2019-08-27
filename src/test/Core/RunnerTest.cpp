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
 * RunnerTest.cpp
 *
 *  Created on: Jul 14, 2017
 *      Author: mircot
 */

#include <boost/test/unit_test.hpp>
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/Object/Aggregator.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Runner/SynchronizedRunner.h"
#include "uavAP/Core/Runner/SynchronizedRunnerMaster.h"
#include <functional>
#include <thread>

class RunnableTestClass: public IRunnableObject, public IAggregatableObject
{
public:

	RunnableTestClass() :
			lastRunStage(RunStage::SYNCHRONIZE)
	{
	}

	bool
	run(RunStage stage)
	{
		lastRunStage = stage;
		return false;
	}

	void
	notifyAggregationOnUpdate(const Aggregator& agg)
	{
	}

	RunStage lastRunStage;
};

class RunnableTestTimeoutClass: public IRunnableObject, public IAggregatableObject
{
public:

	RunnableTestTimeoutClass() :
			lastRunStage(RunStage::SYNCHRONIZE)
	{
	}

	bool
	run(RunStage stage)
	{
		std::this_thread::sleep_for(std::chrono::seconds(2));
		lastRunStage = stage;
		return false;
	}

	void
	notifyAggregationOnUpdate(const Aggregator& agg)
	{
	}

	RunStage lastRunStage;
};

BOOST_AUTO_TEST_SUITE(RunnerTest)

BOOST_AUTO_TEST_CASE(SynchRunner)
{
	auto runObj1 = std::make_shared<RunnableTestClass>();
	auto runObj2 = std::make_shared<RunnableTestClass>();
	Aggregator agg1;
	agg1.add(runObj1);
	agg1.add(runObj2);

	auto runObj3 = std::make_shared<RunnableTestClass>();
	auto runObj4 = std::make_shared<RunnableTestClass>();
	Aggregator agg2;
	agg2.add(runObj3);
	agg2.add(runObj4);

	SynchronizedRunnerMaster master(2);

	std::shared_ptr<SynchronizedRunner> runner1(new SynchronizedRunner);
	std::shared_ptr<SynchronizedRunner> runner2(new SynchronizedRunner);

	BOOST_CHECK_EQUAL((int )runObj1->lastRunStage, (int )RunStage::SYNCHRONIZE);
	BOOST_CHECK_EQUAL((int )runObj2->lastRunStage, (int )RunStage::SYNCHRONIZE);
	BOOST_CHECK_EQUAL((int )runObj3->lastRunStage, (int )RunStage::SYNCHRONIZE);
	BOOST_CHECK_EQUAL((int )runObj4->lastRunStage, (int )RunStage::SYNCHRONIZE);

	std::thread run1(std::bind(&SynchronizedRunner::runSynchronized, runner1, std::ref(agg1)));
	std::thread run2(std::bind(&SynchronizedRunner::runSynchronized, runner2, std::ref(agg2)));

	BOOST_REQUIRE(!master.runStage(RunStage::INIT));

	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	BOOST_CHECK_EQUAL((int )runObj1->lastRunStage, (int )RunStage::INIT);
	BOOST_CHECK_EQUAL((int )runObj2->lastRunStage, (int )RunStage::INIT);
	BOOST_CHECK_EQUAL((int )runObj3->lastRunStage, (int )RunStage::INIT);
	BOOST_CHECK_EQUAL((int )runObj4->lastRunStage, (int )RunStage::INIT);

	BOOST_REQUIRE(!master.runStage(RunStage::NORMAL));

	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	BOOST_CHECK_EQUAL((int )runObj1->lastRunStage, (int )RunStage::NORMAL);
	BOOST_CHECK_EQUAL((int )runObj2->lastRunStage, (int )RunStage::NORMAL);
	BOOST_CHECK_EQUAL((int )runObj3->lastRunStage, (int )RunStage::NORMAL);
	BOOST_CHECK_EQUAL((int )runObj4->lastRunStage, (int )RunStage::NORMAL);

	BOOST_REQUIRE(!master.runStage(RunStage::FINAL));

	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	BOOST_CHECK_EQUAL((int )runObj1->lastRunStage, (int )RunStage::FINAL);
	BOOST_CHECK_EQUAL((int )runObj2->lastRunStage, (int )RunStage::FINAL);
	BOOST_CHECK_EQUAL((int )runObj3->lastRunStage, (int )RunStage::FINAL);
	BOOST_CHECK_EQUAL((int )runObj4->lastRunStage, (int )RunStage::FINAL);

	run1.join();
	run2.join();
}

BOOST_AUTO_TEST_CASE(SynchRunnerTimeout)
{
	auto runObj1 = std::make_shared<RunnableTestTimeoutClass>();
	auto runObj2 = std::make_shared<RunnableTestClass>();
	Aggregator agg1;
	agg1.add(runObj1);
	agg1.add(runObj2);

	auto runObj3 = std::make_shared<RunnableTestClass>();
	auto runObj4 = std::make_shared<RunnableTestClass>();
	Aggregator agg2;
	agg2.add(runObj3);
	agg2.add(runObj4);

	SynchronizedRunnerMaster master(2);

	auto runner1 = std::make_shared<SynchronizedRunner>();
	auto runner2 = std::make_shared<SynchronizedRunner>();

	std::thread run1(
			std::bind(&SynchronizedRunner::runSynchronized, runner1, std::ref(agg1)));
	std::thread run2(
			std::bind(&SynchronizedRunner::runSynchronized, runner2, std::ref(agg2)));

	APLogger::instance()->setLogLevel(LogLevel::NONE);
	//Should timeout due to runObj1
	BOOST_CHECK(master.runStage(RunStage::INIT));
	APLogger::instance()->setLogLevel(LogLevel::WARN);

	run1.join();
	run2.join();
}

BOOST_AUTO_TEST_SUITE_END()
