/*
 * MissionControl.cpp
 *
 *  Created on: Jul 27, 2017
 *      Author: mircot
 */

#include <boost/property_tree/json_parser.hpp>
#include <cpsCore/Synchronization/SynchronizedRunner.h>
#include <cpsCore/Configuration/JsonPopulator.h>
#include "uavAP/MissionControl/MissionControlHelper.h"

int
main(int argc, char** argv)
{
	CPSLogger::instance()->setLogLevel(LogLevel::DEBUG);
	CPSLogger::instance()->setModuleName("MissionControl");
	std::string configPath;
	if (argc == 2)
	{
		configPath = argv[1];
	}
	else
	{
		std::ofstream file;
		file.open("mission_control.json", std::ofstream::out);
		JsonPopulator pop(file);

		pop.populateContainer(MissionControlHelper());
		std::cout << "Populated json" << std::endl;
		return 0;
	}

	Aggregator aggregator = MissionControlHelper::createAggregation(configPath);
	auto sched = aggregator.getOne<IScheduler>();
	sched->setMainThread();

	SynchronizedRunner runner;
	if (runner.runSynchronized(aggregator))
	{
		return 1;
	}

	sched->startSchedule();

	//Terminated -> Cleanup
	aggregator.cleanUp();

	return 0;
}
