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
