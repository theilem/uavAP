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
 * SystemTimeProvider.cpp
 *
 *  Created on: Jul 19, 2017
 *      Author: mircot
 */
#include <boost/thread/pthread/thread_data.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/thread/v2/thread.hpp>
#include "uavAP/Core/TimeProvider/SystemTimeProvider.h"

SystemTimeProvider::SystemTimeProvider()
{
	auto epoch = boost::chrono::system_clock::time_point();
	auto t = boost::chrono::system_clock::to_time_t(epoch);
	chronoEpoch_ = boost::posix_time::from_time_t(t);
}

std::shared_ptr<ITimeProvider>
SystemTimeProvider::create(const boost::property_tree::ptree&)
{
	return std::make_shared<SystemTimeProvider>();
}

TimePoint
SystemTimeProvider::now()
{
	return boost::get_system_time();
}

bool
SystemTimeProvider::waitFor(Duration duration, boost::condition_variable& interrupt,
		boost::unique_lock<boost::mutex>& lock)
{
	boost::chrono::nanoseconds nsec(duration.total_nanoseconds());
	return interrupt.wait_for(lock, nsec) == boost::cv_status::timeout;
}

bool
SystemTimeProvider::waitUntil(TimePoint timePoint, boost::condition_variable& interrupt,
		boost::unique_lock<boost::mutex>& lock)
{
	auto t = boost::posix_time::to_time_t(timePoint);
	boost::chrono::system_clock::time_point tp = boost::chrono::system_clock::from_time_t(t)
			+ boost::chrono::microseconds(timePoint.time_of_day().fractional_seconds());

	return interrupt.wait_until(lock, tp) == boost::cv_status::timeout;
}

void
SystemTimeProvider::notifyAggregationOnUpdate(const Aggregator&)
{
}
