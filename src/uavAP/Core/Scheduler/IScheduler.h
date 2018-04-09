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
 * IScheduler.h
 *
 *  Created on: Jun 29, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_SCHEDULER_ISCHEDULER_H_
#define UAVAP_CORE_SCHEDULER_ISCHEDULER_H_

#include "uavAP/Core/Scheduler/Event.h"
#include "uavAP/Core/Time.h"
#include <functional>

class IScheduler
{
public:

	virtual
	~IScheduler()
	{
	}

	virtual Event
	schedule(const std::function<void
	()>& task, Duration initialFromNow) = 0;

	virtual Event
	schedule(const std::function<void
	()>& task, Duration initialFromNow, Duration period) = 0;

	virtual void
	stop() = 0;

	virtual void
	setMainThread() = 0;

	virtual void
	startSchedule() = 0;
};

#endif /* UAVAP_CORE_SCHEDULER_ISCHEDULER_H_ */
