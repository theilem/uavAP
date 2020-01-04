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
 * Watchdog.cpp
 *
 *  Created on: Aug 9, 2017
 *      Author: mircot
 */

#include <boost/property_tree/json_parser.hpp>

#include <cpsCore/Synchronization/SynchronizedRunnerMaster.h>

#include "uavAP/Watchdog/ProcessMonitor/ProcessMonitor.h"


ProcessMonitor* monitor;

void
sigHandler(int sig)
{
	CPSLOG_DEBUG << "Calling watchdog signal handler with signal: " << sig;
	if (monitor)
	{
		monitor->killAll();
		delete monitor;
	}
	::exit(sig);
}

int
main(int argc, char** argv)
{
	CPSLogger::instance()->setLogLevel(LogLevel::DEBUG);
	CPSLogger::instance()->setModuleName("Watchdog");
	if (argc != 2)
	{
		CPSLOG_ERROR << "Path to config file needed.";
		return 1;
	}

	Configuration conf;
	boost::property_tree::read_json(argv[1], conf);

	signal(SIGINT, sigHandler);
	signal(SIGTERM, sigHandler);

	sigset_t set;
	sigaddset(&set, SIGINT);
	pthread_sigmask(SIG_UNBLOCK, &set, NULL);

	monitor = new ProcessMonitor;
	monitor->configure(conf);

	SynchronizedRunnerMaster runner(monitor->getNumOfProcesses());

	if (!monitor->startAll())
	{
		CPSLOG_ERROR << "Not all processes could be launched. Abort.";
		monitor->killAll();
		return 1;
	}

	CPSLOG_DEBUG << monitor->getNumOfProcesses() << " processes launched.";
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	if (!monitor->checkAlive())
	{
		CPSLOG_ERROR << "Processes did not succeed configuration. Abort.";
		monitor->killAll();
		return 1;
	}

	if (runner.runAllStages())
	{
		monitor->checkAlive();
		CPSLOG_ERROR << "Synchronized running of all stages failed. Abort.";
		monitor->killAll();
		return 1;
	}
	CPSLOG_DEBUG << monitor->getNumOfProcesses() << " processes ran all stages.";
	runner.cleanUp();
	std::string input;
	while (1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		monitor->checkAlive();
	}

	monitor->killAll();

	delete monitor;

	return 0;
}
