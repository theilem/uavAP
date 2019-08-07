/*
 * JsonPopulator.cpp
 *
 *  Created on: Jul 26, 2019
 *      Author: mirco
 */
#include <uavAP/Core/PropertyMapper/JsonPopulator.h>




std::string
JsonPopulator::getString() const
{
	return jsonString_.str();
}
