/*
 * AngleTest.cpp
 *
 *  Created on: Oct 11, 2019
 *      Author: mirco
 */
#include <cpsCore/Utilities/Test/TestInfo.h>
#include "uavAP/Core/SensorData.h"

TEST_CASE("Sensor Data Enum Access")
{
	SensorData data;

	data.position = {1, 2.2, 3};
	data.airSpeed = 7.2;

	CHECK(enumAccess<float>(data, SensorEnum::POSITION_X) == 1.);
	CHECK(enumAccess<int>(data, SensorEnum::POSITION_Y) == 2);

	CHECK(enumAccess<double>(data, SensorEnum::AIR_SPEED) == 7.2);
	CHECK(enumAccess<double>(data, EnumMap<SensorEnum>::convert("air_speed")) == 7.2);
	CHECK(enumAccess<SensorEnum, double>(data, "air_speed") == 7.2);
}
