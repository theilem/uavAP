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
/**
 *  @file         Communication.cpp
 *  @author  Mirco Theile
 *  @date      30 July 2017
 *  @brief      UAV Autopilot Communication Source File
 *
 *  Description
 */

#include <boost/property_tree/json_parser.hpp>

#include "uavAP/Communication/CommunicationHelper.h"
#include "uavAP/Core/Runner/SynchronizedRunner.h"
#include "uavAP/Core/Logging/APLogger.h"

int
main(int argc, char** argv)
{
	APLogger::instance()->setLogLevel(LogLevel::DEBUG);
	APLogger::instance()->setModuleName("Communication");

	CommunicationHelper helper;
	Aggregator aggregator = helper.createAggregation(argv[1]);
	auto sched = aggregator.getOne<IScheduler>();
	sched->setMainThread();

	SynchronizedRunner runner;
	if (runner.runSynchronized(aggregator))
	{
		APLOG_ERROR << "Failed to run Synchronized.";
		return 1;
	}

	sched->startSchedule();

	return 0;
}
