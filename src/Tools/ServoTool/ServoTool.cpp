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
#include "uavAP/API/ap_ext/ApExtManager.h"
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/Runner/SimpleRunner.h"
#include "uavAP/Core/Scheduler/MultiThreadingScheduler.h"
#include "uavAP/Core/TimeProvider/SystemTimeProvider.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/Core/IPC/IPC.h"

int
main(int argc, char** argv)
{
	APLogger::instance()->setLogLevel(LogLevel::DEBUG);
	auto ipc = std::make_shared<IPC>();
	auto tp = std::make_shared<SystemTimeProvider>();
	auto sched = std::make_shared<MultiThreadingScheduler>();
	auto agg = Aggregator::aggregate(
	{ ipc, tp, sched });

	auto servoPub = ipc->publish<ApExtManager::OutPWM>("servo_out");

	SimpleRunner runner(agg);
	runner.runAllStages();
	ApExtManager::OutPWM data;

	for (int i = 0; i < 7; ++i)
	{
		data.ch[i] = 6000;
	}

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
			servoPub.publish(data);
		}
		else if (input.compare("0") == 0)
		{
			std::cout << "Set Channel 0:" << std::endl;
			std::cin >> data.ch[0];
		}
		else if (input.compare("1") == 0)
		{
			std::cout << "Set Channel 1:" << std::endl;
			std::cin >> data.ch[1];
		}
		else if (input.compare("2") == 0)
		{
			std::cout << "Set Channel 2:" << std::endl;
			std::cin >> data.ch[2];
		}
		else if (input.compare("3") == 0)
		{
			std::cout << "Set Channel 3:" << std::endl;
			std::cin >> data.ch[3];
		}
		else if (input.compare("4") == 0)
		{
			std::cout << "Set Channel 4:" << std::endl;
			std::cin >> data.ch[4];
		}
		else if (input.compare("5") == 0)
		{
			std::cout << "Set Channel 5:" << std::endl;
			std::cin >> data.ch[5];
		}
		else if (input.compare("6") == 0)
		{
			std::cout << "Set Channel 6:" << std::endl;
			std::cin >> data.ch[6];
		}
	}

	return 0;
}
