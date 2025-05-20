/**
 *  @file         Communication.cpp
 *  @author  Mirco Theile
 *  @date      30 July 2017
 *  @brief      UAV Autopilot Communication Source File
 *
 *  Description
 */


#include <cpsCore/Synchronization/SynchronizedRunner.h>
#include <cpsCore/Configuration/JsonPopulator.h>

#include "uavAP/Communication/CommunicationHelper.h"

int
main(int argc, char** argv)
{
	CPSLogger::instance()->setLogLevel(LogLevel::DEBUG);
	CPSLogger::instance()->setModuleName("Communication");
	std::string configPath;
	if (argc == 2)
	{
		configPath = argv[1];
	}
	else
	{
		JsonPopulator pop;

		pop.populateContainer<CommunicationHelper>();
		pop.toFile("communication.json");
		std::cout << "Populated json" << std::endl;
		return 0;
	}

	Aggregator aggregator = CommunicationHelper::createAggregation(configPath);
	auto sched = aggregator.getOne<IScheduler>();
	sched->setMainThread();

	SynchronizedRunner runner;
	if (runner.runSynchronized(aggregator))
	{
		CPSLOG_ERROR << "Failed to run Synchronized.";
		return 1;
	}

	sched->startSchedule();

	//Terminated -> Cleanup
	aggregator.cleanUp();

	return 0;
}
