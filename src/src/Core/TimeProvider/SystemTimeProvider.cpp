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
#include <boost/thread/thread_time.hpp>
#include "uavAP/Core/TimeProvider/SystemTimeProvider.h"

SystemTimeProvider::SystemTimeProvider()
{
	auto epoch = std::chrono::system_clock::time_point();
	auto t = std::chrono::system_clock::to_time_t(epoch);
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
SystemTimeProvider::waitFor(Duration duration, std::condition_variable& interrupt,
		std::unique_lock<std::mutex>& lock)
{
	std::chrono::nanoseconds nsec(duration.total_nanoseconds());
	return interrupt.wait_for(lock, nsec) == std::cv_status::timeout;
}

bool
SystemTimeProvider::waitUntil(TimePoint timePoint, std::condition_variable& interrupt,
		std::unique_lock<std::mutex>& lock)
{
	auto t = boost::posix_time::to_time_t(timePoint);
	std::chrono::system_clock::time_point tp = std::chrono::system_clock::from_time_t(t)
			+ std::chrono::microseconds(timePoint.time_of_day().fractional_seconds());

	return interrupt.wait_until(lock, tp) == std::cv_status::timeout;
}

void
SystemTimeProvider::notifyAggregationOnUpdate(const Aggregator&)
{
}
