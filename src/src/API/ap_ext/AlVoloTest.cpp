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
 * AlVoloTest.cpp
 *
 *  Created on: Sep 2, 2017
 *      Author: mircot
 */

#include <boost/thread.hpp>
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/Time.h"
#include "uavAP/API/ap_ext/ap_ext.h"
#include <csignal>
#include <iostream>
#include <thread>

void
signalHandler(int signal)
{
	std::cout << "Signal handler called" << std::endl;
	ap_ext_teardown();
	exit(signal);
}

static bool stop = false;

void
command()
{
	for (;;)
	{
		std::string c;
		std::cin >> c;
		if (c.compare("q") == 0)
		{
			stop = true;
			return;
		}
	}
}


int
main(int argc, char** argv)
{
	APLogger::instance()->setLogLevel(LogLevel::DEBUG);
	std::signal(SIGINT, signalHandler);
	ap_ext_setup();

	unsigned long pwm[7];

	std::thread commandThread(&command);

	for (;;)
	{
		ap_ext_actuate(&pwm[0], 7);

		std::cout << "============================================================" << std::endl;
		for (int i = 0; i < 7; ++i)
		{
			std::cout << "Channel " << i << ": " << pwm[i] << std::endl;
		}
		std::cout << std::endl;

		boost::this_thread::sleep(Seconds(1));
		if (stop)
		{
			ap_ext_teardown();
			commandThread.join();
			return 0;
		}
	}



}
