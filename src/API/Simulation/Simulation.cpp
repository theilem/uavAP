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
 * Simulation.cpp
 *
 *  Created on: Jul 28, 2017
 *      Author: mircot
 */
#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "uavAP/API/Simulation/SimulationHelper.h"
#include "uavAP/Core/IDC/Serial/SerialIDC.h"
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/Runner/SimpleRunner.h"
#include "uavAP/Core/Scheduler/MultiThreadingScheduler.h"
#include "uavAP/Core/TimeProvider/SystemTimeProvider.h"
#include <iostream>

int
main(int argc, char** argv)
{
	APLogger::instance()->setLogLevel(LogLevel::DEBUG);
	if (argc != 2)
	{
		APLOG_ERROR << "Path to config file needed.";
		return 1;
	}

	Aggregator aggregator = SimulationHelper::createAggregation(argv[1]);
	SimpleRunner run(aggregator);

	if (run.runAllStages())
	{
		return 1;
	}

	std::string input;
	while (input.compare("q") != 0)
	{
		input.clear();
		std::cin >> input;
	}

	return 0;
}
