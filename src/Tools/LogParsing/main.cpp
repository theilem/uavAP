//
// Created by mirco on 24.06.20.
//

#include <cpsCore/Configuration/JsonPopulator.h>
#include <cpsCore/Synchronization/SimpleRunner.h>
#include "LogParsingHelper.h"

int
main(int argc, char** argv)
{

	CPSLogger::instance()->setLogLevel(LogLevel::DEBUG);
	CPSLogger::instance()->setModuleName("LogParser");
	std::string configPath;
	if (argc == 2)
	{
		configPath = argv[1];
	}
	else
	{
		JsonPopulator pop;

		pop.populateContainer<LogParsingHelper>();
		pop.toFile("log_parser.json");
		std::cout << "Populated json" << std::endl;
		return 0;
	}

	Aggregator aggregator = LogParsingHelper::createAggregation(configPath);
	auto sched = aggregator.getOne<IScheduler>();
	sched->setMainThread();

	SimpleRunner runner(aggregator);
	if (runner.runAllStages())
	{
		CPSLOG_ERROR << "Something went wrong";
		return 1;
	}

	sched->startSchedule();

	CPSLOG_DEBUG << "Done";

	//Terminated -> Cleanup
	aggregator.cleanUp();

	return 0;


}