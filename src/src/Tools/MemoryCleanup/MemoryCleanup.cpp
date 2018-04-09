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
 * MemoryCleanup.cpp
 *
 *  Created on: Nov 26, 2017
 *      Author: mircot
 */
#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include "uavAP/Core/Logging/APLogger.h"
#include <vector>


const static std::vector<std::string> sharedMem =
{
		"sensor_data",
		"actuation"
};

const static std::vector<std::string> messageQueue =
{
		"data_fc_com",
		"data_com_fc",
		"data_mc_com",
		"data_com_mc",
		"data_api_com",
		"data_com_api"
};

void
cleanSharedMemory(const std::string& name)
{
	boost::interprocess::shared_memory_object obj;
	obj.remove(name.c_str());
}

void
cleanMessageQueue(const std::string& name)
{
	boost::interprocess::message_queue::remove(name.c_str());
}

void
cleanAll()
{
	for (auto& it : sharedMem)
	{
		cleanSharedMemory(it);
	}
	for (auto& it : messageQueue)
	{
		cleanMessageQueue(it);
	}
}

int
main(int argc, char** argv)
{
	APLogger::instance()->setLogLevel(LogLevel::DEBUG);

	std::string input;

	while (1)
	{
		input.clear();
		std::cin >> input;
		if (input.compare("q") == 0)
		{
			break;
		}
		else if (input.compare("all") == 0)
		{
			cleanAll();
		}
		else if (input.compare("shared") == 0)
		{
			std::string in;
			std::cin >> in;
			cleanSharedMemory(in);
		}
		else if (input.compare("queue") == 0)
		{
			std::string in;
			std::cin >> in;
			cleanMessageQueue(in);
		}

	}

	return 0;
}

