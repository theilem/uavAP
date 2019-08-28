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
 * EventBody.h
 *
 *  Created on: Jul 20, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_SCHEDULER_EVENTBODY_H_
#define UAVAP_CORE_SCHEDULER_EVENTBODY_H_

#include <functional>
#include <boost/optional.hpp>
#include "uavAP/Core/Time.h"
#include <atomic>
#include <condition_variable>
#include <thread>

struct EventBody
{

	std::function<void
	()> body;
	boost::optional<Duration> period;
	std::atomic_bool isCanceled;
	std::atomic_bool isStarted;
	std::atomic_bool missedDeadline;

	std::mutex executionMutex;
	std::thread periodicThread;
	std::condition_variable intervalCondition;

	EventBody(const std::function<void
	()>& b);

	EventBody(const std::function<void
	()>& b, const Duration& p);

};

#endif /* UAVAP_CORE_SCHEDULER_EVENTBODY_H_ */
