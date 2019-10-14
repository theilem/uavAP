/*
 * JsonPopulation.cpp
 *
 *  Created on: Aug 28, 2019
 *      Author: mirco
 */
#include <uavAP/Core/PropertyMapper/JsonPopulator.h>
#include <uavAP/Core/Scheduler/MultiThreadingScheduler.h>
#include <uavAP/FlightControl/Controller/PIDController/ManeuverRatePIDController/ManeuverRatePIDController.h>
#include <uavAP/MissionControl/ManeuverPlanner/ManeuverPlanner.h>
#include <fstream>
#include <string>




int
main(int argc, char** argv)
{
	std::string filename = "a.json";
	if (argc >= 2)
	{
		filename = argv[1];
	}

	std::ofstream file;

	file.open(filename, std::ofstream::out);

	JsonPopulator pop(file);

	pop.populate<ManeuverRatePIDController>();

	return 0;


}
