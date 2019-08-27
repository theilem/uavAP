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
 * EventBody.cpp
 *
 *  Created on: Jul 20, 2017
 *      Author: mircot
 */

#include "uavAP/Core/Scheduler/EventBody.h"

EventBody::EventBody(const std::function<void
()>& b, const sched_param* param) :
		body(b), isCanceled(false), isStarted(false), missedDeadline(false)
{
	if (param)
		pthread_setschedparam(periodicThread.native_handle(), SCHED_FIFO, param);
}

EventBody::EventBody(const std::function<void
()>& b, const Duration& p, const sched_param* param) :
		body(b), period(p), isCanceled(false), isStarted(false), missedDeadline(false)
{
	if (param)
		pthread_setschedparam(periodicThread.native_handle(), SCHED_FIFO, param);
}
