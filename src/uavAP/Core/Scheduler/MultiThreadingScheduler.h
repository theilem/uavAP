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
 * MultiThreadedScheduler.h
 *
 *  Created on: Jul 19, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_SCHEDULER_MULTITHREADINGSCHEDULER_H_
#define UAVAP_CORE_SCHEDULER_MULTITHREADINGSCHEDULER_H_
#include <boost/optional/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/thread.hpp>
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/Runner/IRunnableObject.h"
#include "uavAP/Core/Scheduler/EventBody.h"
#include "uavAP/Core/Scheduler/IScheduler.h"
#include "uavAP/Core/TimeProvider/ITimeProvider.h"

class MultiThreadingScheduler: public IScheduler, public IAggregatableObject, public IRunnableObject
{
public:

	/**
	 * @brief
	 */
	MultiThreadingScheduler();

	~MultiThreadingScheduler();

	static std::shared_ptr<IScheduler>
	create(const boost::property_tree::ptree& conf);

	Event
	schedule(const std::function<void
	()>& task, Duration initialFromNow) override;

	Event
	schedule(const std::function<void
	()>& task, Duration initialFromNow, Duration period) override;

	void
	stop() override;

	bool
	run(RunStage stage) override;

	void
	setMainThread() override;

	void
	startSchedule() override;

	void
	notifyAggregationOnUpdate(Aggregator& agg) override;

private:

	void
	runSchedule();

	void
	sleepUntil(TimePoint timepoint);

	using EventMap = std::multimap<Duration, std::shared_ptr<EventBody> >;

	EventMap::value_type
	createSchedule(Duration start, std::shared_ptr<EventBody> body);

	void
	periodicTask(std::shared_ptr<EventBody> body);

	void
	handleMissedDeadline(std::shared_ptr<EventBody> body);

	EventMap events_;

	ObjectHandle<ITimeProvider> timeProvider_;

	boost::thread invokerThread_;

	boost::condition_variable wakeupCondition_;
	boost::mutex eventsMutex_;

	bool started_;
	TimePoint startingTime_;

	bool mainThread_;
};

#endif /* UAVAP_CORE_SCHEDULER_MULTITHREADINGSCHEDULER_H_ */
