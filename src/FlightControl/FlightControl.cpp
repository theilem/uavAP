/*
 * Control.cpp
 *
 *  Created on: Jun 23, 2017
 *      Author: mircot
 */

#include <cpsCore/Synchronization/SynchronizedRunner.h>
#include <cpsCore/Configuration/JsonPopulator.h>
#include "uavAP/FlightControl/FlightControlHelper.h"


int
main(int argc, char** argv)
{
	CPSLogger::instance()->setLogLevel(LogLevel::WARN);
	CPSLogger::instance()->setModuleName("FlightControl");
	std::string configPath;
	if (argc == 2)
	{
		configPath = argv[1];
	}
	else
	{
		JsonPopulator pop;

		pop.populateContainer<FlightControlHelper>();
		pop.toFile("flight_control.json");
		std::cout << "Populated json" << std::endl;
		return 0;
	}

	Aggregator aggregator = FlightControlHelper::createAggregation(configPath);
	auto sched = aggregator.getOne<IScheduler>();
	sched->setMainThread();

	CPSLOG_DEBUG << "Run synchronized";
	SynchronizedRunner runner;
	if (runner.runSynchronized(aggregator))
	{
		CPSLOG_ERROR << "Something went wrong";
		return 1;
	}

	sched->startSchedule();

	//Terminated -> Cleanup
	aggregator.cleanUp();

	return 0;
}
