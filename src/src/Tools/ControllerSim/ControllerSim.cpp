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
 * ControllerSim.cpp
 *
 *  Created on: Sep 2, 2017
 *      Author: mircot
 */
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/Runner/SimpleRunner.h"
#include "uavAP/Core/Scheduler/MultiThreadingScheduler.h"
#include "uavAP/Core/TimeProvider/SystemTimeProvider.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/Core/IPC/IPC.h"
#include <iostream>

int
main(int argc, char** argv)
{
	APLogger::instance()->setLogLevel(LogLevel::DEBUG);
	auto ipc = std::make_shared<IPC>();
	auto tp = std::make_shared<SystemTimeProvider>();
	auto sched = std::make_shared<MultiThreadingScheduler>();
	auto agg = Aggregator::aggregate(
	{ ipc, tp, sched });

	auto controlPub = ipc->publish<ControllerOutput>("actuation");

	SimpleRunner runner(agg);
	runner.runAllStages();
	ControllerOutput data;

	std::string input;

	while (1)
	{
		input.clear();
		std::cin >> input;
		if (input.compare("q") == 0)
		{
			break;
		}
		else if (input.compare("s") == 0)
		{
			std::cout << "Send data" << std::endl;
			controlPub.publish(data);
		}
		else if (input.compare("r") == 0)
		{
			std::cout << "set roll" << std::endl;
			std::cin >> data.rollOutput;
		}

		else if (input.compare("p") == 0)
		{
			std::cout << "set pitch" << std::endl;
			std::cin >> data.pitchOutput;
		}

		else if (input.compare("y") == 0)
		{
			std::cout << "set yaw" << std::endl;
			std::cin >> data.yawOutput;
		}

		else if (input.compare("t") == 0)
		{
			std::cout << "set throttle" << std::endl;
			std::cin >> data.throttleOutput;
		}
	}

	return 0;
}
