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
 * Control.cpp
 *
 *  Created on: Jun 23, 2017
 *      Author: mircot
 */

#include <boost/property_tree/json_parser.hpp>
#include <cpsCore/Synchronization/SynchronizedRunner.h>
#include "uavAP/FlightControl/FlightControlHelper.h"


int
main(int argc, char** argv)
{
	CPSLogger::instance()->setLogLevel(LogLevel::DEBUG);
	CPSLogger::instance()->setModuleName("FlightControl");
	std::string configPath;
	if (argc == 2)
	{
		configPath = argv[1];
	}

	FlightControlHelper helper;
	Aggregator aggregator = helper.createAggregation(configPath);
	auto sched = aggregator.getOne<IScheduler>();
	sched->setMainThread();

	CPSLOG_DEBUG << "Run synchronized";
	SynchronizedRunner runner;
	if (runner.runSynchronized(aggregator))
	{
		CPSLOG_ERROR << "Something went wrong";
		return 1;
	}

	sched->startSchedule();

	//Terminated -> Cleanup
	aggregator.cleanUp();

	return 0;
}
