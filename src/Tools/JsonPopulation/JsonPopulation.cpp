/*
 * JsonPopulation.cpp
 *
 *  Created on: Aug 28, 2019
 *      Author: mirco
 */
#include <fstream>
#include <string>

#include <cpsCore/Configuration/JsonPopulator.h>




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

//	pop.populate<Geofencing, ConstRollRateModel>();

	return 0;


}
