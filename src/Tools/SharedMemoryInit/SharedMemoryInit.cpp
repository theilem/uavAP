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
 * OutputListener.cpp
 *
 *  Created on: Jul 27, 2017
 *      Author: mircot
 */

#include "uavAP/Core/SensorData.h"

#include <iostream>
#include <cpsCore/Utilities/IPC/IPC.h>
#include <cpsCore/Utilities/SignalHandler/SignalHandler.h>

int
main(int argc, char** argv)
{
	CPSLogger::instance()->setLogLevel(LogLevel::DEBUG);
	auto ipc = std::make_shared<IPC>();

	SensorData sd;
	sd.timestamp = Clock::now();
	sd.hasGPSFix = true;
	auto pub = ipc->publish<SensorData>("sensor_data");
	pub.publish(sd);

	std::string input;
	while (1)
	{
		input.clear();
		std::cin >> input;
		if (input == "q")
		{
			break;
		}
		if (input == "u")
		{
			sd.timestamp = Clock::now();
			pub.publish(sd);
		}
	}

	return 0;
}