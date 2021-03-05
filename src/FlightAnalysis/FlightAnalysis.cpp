/*
 * FlightAnalysis.cpp
 *
 *  Created on: Jul 22, 2018
 *      Author: simonyu
 */
#include "cpsCore/Logging/CPSLogger.h"
#include "cpsCore/Configuration/JsonPopulator.h"
#include "cpsCore/Synchronization/SynchronizedRunner.h"
#include "uavAP/FlightAnalysis/FlightAnalysisHelper.h"

int
main(int argc, char** argv)
{
	CPSLogger::instance()->setLogLevel(LogLevel::DEBUG);
	CPSLogger::instance()->setModuleName("FlightAnalysis");
	std::string configPath;
	if (argc == 2)
	{
		configPath = argv[1];
	}
	else
	{
		std::ofstream file;
		file.open("flight_analysis.json", std::ofstream::out);
//		JsonPopulator pop(file);
//
//		pop.populateContainer(FlightAnalysisHelper());
		std::cout << "Populated json" << std::endl;
		return 0;
	}

	Aggregator aggregator = FlightAnalysisHelper::createAggregation(configPath);
	auto sched = aggregator.getOne<IScheduler>();
	sched->setMainThread();

	CPSLOG_DEBUG << "Run synchronized";
	SynchronizedRunner runner;
	if (runner.runSynchronized(aggregator))
	{
		CPSLOG_ERROR << "Failed during run stages";
		return 1;
	}

	sched->startSchedule();

	//Terminated -> Cleanup
	aggregator.cleanUp();

	return 0;
}
