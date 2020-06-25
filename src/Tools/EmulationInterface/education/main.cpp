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
 * EmulationInterface.cpp
 *
 *  Created on: Jan 22, 2018
 *      Author: mircot
 */
#include <boost/property_tree/json_parser.hpp>
#include "EduInterfaceHelper.h"
#include <uavAP/Core/Runner/SimpleRunner.h>

#include "uavAP/Core/Scheduler/MultiThreadingScheduler.h"
#include "uavAP/Core/Logging/APLogger.h"

int
main(int argc, char** argv)
{
	APLogger::instance()->setLogLevel(LogLevel::WARN);
	APLogger::instance()->setModuleName("EmulationInterface");
	std::string configPath;
	if (argc == 2)
	{
		configPath = argv[1];
	}
	else
	{
		APLOG_DEBUG << "Trying default config path: /usr/local/config/edu.json";
		configPath = "/usr/local/config/edu.json";
	}

	auto agg = EduInterfaceHelper::createAggregation(configPath);

	auto sched = agg.getOne<IScheduler>();
	sched->setMainThread();

	SimpleRunner runner(agg);
	if (runner.runAllStages())
	{
		APLOG_ERROR << "Errors occured in run all stages";
		return 1;
	}

	sched->startSchedule();
	return 0;
}
