/*
 * EmulationInterface.cpp
 *
 *  Created on: Jan 22, 2018
 *      Author: mircot
 */

#include "EmulationInterfaceHelper.h"

#include <cpsCore/Configuration/JsonPopulator.h>
#include <fstream>
#include <cpsCore/Synchronization/SynchronizedRunner.h>

int
main(int argc, char** argv)
{
	CPSLogger::instance()->setLogLevel(LogLevel::WARN);
	CPSLogger::instance()->setModuleName("EmulationInterface");
	std::string configPath;
	if (argc == 2)
	{
		configPath = argv[1];
	}
	else
	{
		std::ofstream file;
		file.open("emulation_interface.json", std::ofstream::out);
		JsonPopulator pop(file);

		pop.populateContainer(EmulationInterfaceHelper());
		std::cout << "Populated json" << std::endl;
		return 0;
	}

	auto agg = EmulationInterfaceHelper::createAggregation(configPath);

	auto sched = agg.getOne<IScheduler>();
	sched->setMainThread();

	SynchronizedRunner runner;
	if (runner.runSynchronized(agg))
	{
		CPSLOG_ERROR << "Errors occured in run all stages";
		return 1;
	}

	sched->startSchedule();

	agg.cleanUp();
	return 0;
}
