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
 * MicroSimulator.cpp
 *
 *  Created on: Jun 29, 2017
 *      Author: mircot
 */

#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/Scheduler/MicroSimulator.h"
#include <utility>

MicroSimulator::MicroSimulator() :
		now_(), runs_(0), stopOnWait_(false), waitCondition_(
				nullptr), waitReleased_(false)
{
}

Event
MicroSimulator::schedule(const std::function<void
()>& task, Duration initialFromNow)
{
	auto body = std::make_shared<EventBody>(task);
	events_.insert(std::make_pair(now_ + initialFromNow, body));
	return Event(body);
}

Event
MicroSimulator::schedule(const std::function<void
()>& task, Duration initialFromNow, Duration period)
{
	auto body = std::make_shared<EventBody>(task, period);
	events_.insert(std::make_pair(now_ + initialFromNow, body));
	return Event(body);
}

void
MicroSimulator::stop()
{
	//Do nothing;
}

void
MicroSimulator::notifyAggregationOnUpdate(const Aggregator&)
{
}

int
MicroSimulator::simulate(Duration duration)
{
	runs_ = 0;
	TimePoint endSim = now_ + duration;
	while (now_ <= endSim)
	{
		if (events_.empty())
		{
			break;
		}

		if (events_.begin()->first > endSim)
		{
			break;
		}

		now_ = events_.begin()->first;
		events_.begin()->second->body();
		if (events_.begin()->second->period)
		{
			schedule(events_.begin()->second->body, *events_.begin()->second->period,
					*events_.begin()->second->period);
		}
		++runs_;
		events_.erase(events_.begin());
	}

	now_ = endSim;
	return runs_;
}

void
MicroSimulator::stopOnWait()
{
	stopOnWait_ = true;
}

void
MicroSimulator::setMainThread()
{
}

void
MicroSimulator::startSchedule()
{
}

void
MicroSimulator::releaseWait()
{
	std::unique_lock<std::mutex> lock(waitCondMutex_);
	if (!waitCondition_)
	{
		APLOG_WARN << "Nobody waiting on release";
		return;
	}
	waitReleased_ = true;
	waitCondition_->notify_all();
}

TimePoint
MicroSimulator::now()
{
	return now_;
}

bool
MicroSimulator::waitFor(Duration duration, std::condition_variable& interrupt,
		std::unique_lock<std::mutex>& lock)
{
	if (stopOnWait_)
	{
		waitCondition_ = &interrupt;
		waitCondition_->wait(lock);
	}
	bool waitReleased = waitReleased_;
	waitReleased_ = false;
	if (waitReleased)
		now_ += duration;
	return waitReleased;
}

bool
MicroSimulator::waitUntil(TimePoint timePoint, std::condition_variable& interrupt,
		std::unique_lock<std::mutex>& lock)
{
	if (stopOnWait_)
	{

		waitCondition_ = &interrupt;
		waitCondition_->wait(lock);
	}
	bool waitReleased = waitReleased_;
	waitReleased_ = false;
	if (waitReleased)
		now_ = timePoint;
	return waitReleased;
}
