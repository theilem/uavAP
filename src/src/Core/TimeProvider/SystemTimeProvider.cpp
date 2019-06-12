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

SystemTimeProvider::SystemTimeProvider():chronoEpoch_(TimePoint())
{
}

std::shared_ptr<ITimeProvider>
SystemTimeProvider::create(const Configuration&)
{
	return std::make_shared<SystemTimeProvider>();
}

TimePoint
SystemTimeProvider::now()
{
	return Clock::now();
}

bool
SystemTimeProvider::waitFor(Duration duration, std::condition_variable& interrupt,
		std::unique_lock<std::mutex>& lock)
{
	return interrupt.wait_for(lock, duration) == std::cv_status::timeout;
}

bool
SystemTimeProvider::waitUntil(TimePoint timePoint, std::condition_variable& interrupt,
		std::unique_lock<std::mutex>& lock)
{
	return interrupt.wait_until(lock, timePoint) == std::cv_status::timeout;
}

void
SystemTimeProvider::notifyAggregationOnUpdate(const Aggregator&)
{
}
