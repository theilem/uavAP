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
 * MultiThreadingScheduler.cpp
 *
 *  Created on: Jul 19, 2017
 *      Author: mircot
 */

#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/Scheduler/MultiThreadingScheduler.h"
#include <utility>

MultiThreadingScheduler::MultiThreadingScheduler() :
		started_(false), mainThread_(false)
{
}

MultiThreadingScheduler::~MultiThreadingScheduler()
{
	if (started_)
		stop();
}

std::shared_ptr<IScheduler>
MultiThreadingScheduler::create(const boost::property_tree::ptree&)
{
	return std::make_shared<MultiThreadingScheduler>();
}

Event
MultiThreadingScheduler::schedule(const std::function<void
()>& task, Duration initialFromNow)
{
	auto body = std::make_shared<EventBody>(task);
	auto element = createSchedule(initialFromNow, body);

	std::unique_lock<std::mutex> lock(eventsMutex_);
	events_.insert(element);
	wakeupCondition_.notify_all();
	return Event(body);
}

Event
MultiThreadingScheduler::schedule(const std::function<void
()>& task, Duration initialFromNow, Duration period)
{
	auto body = std::make_shared<EventBody>(task, period);
	auto element = createSchedule(initialFromNow, body);

	std::unique_lock<std::mutex> lock(eventsMutex_);
	events_.insert(element);
	wakeupCondition_.notify_all();
	return Event(body);
}

void
MultiThreadingScheduler::stop()
{
	std::unique_lock<std::mutex> lock(eventsMutex_);
	started_ = false;
	wakeupCondition_.notify_all();
	lock.unlock();
	invokerThread_.join();
}

bool
MultiThreadingScheduler::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
		if (!timeProvider_.isSet())
		{
			APLOG_ERROR << "TimeProvider missing.";
			return true;
		}
		break;
	case RunStage::NORMAL:
		break;
	case RunStage::FINAL:
		started_ = true;
		if (!mainThread_)
			invokerThread_ = std::thread(
					boost::bind(&MultiThreadingScheduler::runSchedule, this));

		break;
	default:
		break;
	}
	return false;
}

void
MultiThreadingScheduler::notifyAggregationOnUpdate(const Aggregator& agg)
{
	timeProvider_.setFromAggregationIfNotSet(agg);
}

void
MultiThreadingScheduler::runSchedule()
{
	auto timeProvider = timeProvider_.get();
	if (!timeProvider)
	{
		APLOG_ERROR << "Time Provider missing in Scheduler. Abort.";
		return;
	}

	APLOG_TRACE << "Starting Scheduling";

	std::unique_lock<std::mutex> lock(eventsMutex_);

	startingTime_ = timeProvider->now();
	for (;;)
	{
		TimePoint now = timeProvider->now();
		while (!events_.empty())
		{
			if (startingTime_ + events_.begin()->first > now)
				break;
			APLOG_TRACE << "Executing task";
			auto it = events_.begin();

			auto eventBody = it->second;

			if (eventBody->period)
			{
				//Is a periodic task
				//Check if it is already running
				if (eventBody->isStarted.load())
				{
					//Thread is already started
					std::unique_lock<std::mutex> lock(eventBody->executionMutex,
							std::try_to_lock);
					if (lock)
					{
						//The task is waiting at the condition variable
						eventBody->intervalCondition.notify_one(); //Only one should be waiting on it
						lock.unlock();
					}
					else
					{
						//Task didn't finish execution yet
						handleMissedDeadline(eventBody);
						continue;
					}
					//Check if isCanceled
					if (eventBody->isCanceled.load())
					{
						eventBody->periodicThread.join();
						events_.erase(events_.begin());
						continue;
					}
				}
				else
				{
					//Thread not started yet
					eventBody->periodicThread = std::thread(
							boost::bind(&MultiThreadingScheduler::periodicTask, this, eventBody));
				}
				//Reschedule Task
				auto element = std::make_pair(it->first + *eventBody->period, eventBody);
				events_.insert(element);
			}
			else
			{
				//Not a periodic thread
				if (!it->second->isCanceled.load())
				{
					std::thread(it->second->body).detach();
				}
			}
			//Remove current schedule from events as it was handeled
			events_.erase(events_.begin());
		}
		if (!events_.empty())
		{
			timeProvider->waitUntil(startingTime_ + events_.begin()->first, wakeupCondition_, lock);
		}
		else
		{
			wakeupCondition_.wait(lock);
		}
		if (!started_)
		{
			APLOG_DEBUG << "Scheduler was stopped.";
			return;
		}
	}
}

MultiThreadingScheduler::EventMap::value_type
MultiThreadingScheduler::createSchedule(Duration start, std::shared_ptr<EventBody> body)
{
	auto tp = timeProvider_.get();
	if (!tp)
	{
		APLOG_ERROR << "TimeProvider missing. Cannot Schedule.";
		return std::make_pair(start, body);
	}

	Duration fromStart = start;

	if (started_)
	{
		auto now = tp->now();
		fromStart += now - startingTime_;
	}

	return std::make_pair(fromStart, body);
}

void
MultiThreadingScheduler::periodicTask(std::shared_ptr<EventBody> body)
{
	body->isStarted.store(true);
	std::unique_lock<std::mutex> lock(body->executionMutex);
	while (!body->isCanceled.load())
	{
		body->body();
		body->intervalCondition.wait(lock);
	}
	body->isStarted.store(false);
}

void
MultiThreadingScheduler::setMainThread()
{
	mainThread_ = true;
}

void
MultiThreadingScheduler::startSchedule()
{
	if (!mainThread_)
	{
		APLOG_ERROR
				<< "Scheduler not configured to be main thread. Invoker thread probably running.";
		return;
	}
	runSchedule();
}

void
MultiThreadingScheduler::handleMissedDeadline(std::shared_ptr<EventBody> body)
{
	APLOG_WARN << body->body.target_type().name() << " Missed Deadline. Waiting...";
	//Reschedule Task
	auto element = std::make_pair(events_.begin()->first + *body->period, body);
	events_.insert(element);
	events_.erase(events_.begin());
}
