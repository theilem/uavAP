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
 * SchedulerRTTest.cpp
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

BOOST_AUTO_TEST_SUITE(SchedulerRTTest)

void
task(Duration period, TimePoint& lastCheckin, uint64_t& sum, int& count)
{
	TimePoint now = boost::get_system_time();
	if (!lastCheckin.is_not_a_date_time())
	{
		Duration diff = now - lastCheckin;
		//Check that no execution exeeds 5%
		BOOST_CHECK_CLOSE((double )diff.total_microseconds(), (double )period.total_microseconds(),
				5);
		sum += abs(diff.total_microseconds() - period.total_microseconds());
		++count;
	}
	lastCheckin = now;
}

BOOST_AUTO_TEST_CASE(MultiThreadedRTTest001)
{
	auto tp = std::make_shared<SystemTimeProvider>();
	auto sched = std::make_shared<MultiThreadingScheduler>();
	auto runner = std::make_shared<AggregatableRunner>();
	Aggregator::aggregate( { tp, sched, runner });
	APLogger::instance()->setLogLevel(LogLevel::WARN);

	TimePoint checkIn10msec;
	TimePoint checkIn25msec;
	TimePoint checkIn77msec;

	uint64_t sum10msec = 0;
	uint64_t sum25msec = 0;
	uint64_t sum77msec = 0;

	int count10msec = 0;
	int count25msec = 0;
	int count77msec = 0;

	sched->schedule(
			boost::bind(task, Milliseconds(10), boost::ref(checkIn10msec), boost::ref(sum10msec),
					boost::ref(count10msec)), Milliseconds(0), Milliseconds(10));
	sched->schedule(
			boost::bind(task, Milliseconds(25), boost::ref(checkIn25msec), boost::ref(sum25msec),
					boost::ref(count25msec)), Milliseconds(0), Milliseconds(25));
	sched->schedule(
			boost::bind(task, Milliseconds(77), boost::ref(checkIn77msec), boost::ref(sum77msec),
					boost::ref(count77msec)), Milliseconds(0), Milliseconds(77));

	runner->runAllStages();

	boost::this_thread::sleep(Seconds(5));

	sched->stop();

	//Check average
	double tolerance = 0.02;
	BOOST_CHECK_LE((double )sum10msec / count10msec, tolerance * 10000);
	BOOST_CHECK_LE((double )sum25msec / count25msec, tolerance * 25000);
	BOOST_CHECK_LE((double )sum77msec / count77msec, tolerance * 77000);

	APLogger::instance()->setLogLevel(LogLevel::WARN);
}

BOOST_AUTO_TEST_SUITE_END()
