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

	CHECK(enumAccess<float>(data, SensorEnum::POSITION_X) == 1.f);
	CHECK(enumAccess<int>(data, SensorEnum::POSITION_Y) == 2);

	CHECK(enumAccess<FloatingType>(data, SensorEnum::AIR_SPEED) == FloatingType(7.2));
	CHECK(enumAccess<FloatingType>(data, EnumMap<SensorEnum>::convert("air_speed")) == FloatingType(7.2));
	CHECK(enumAccess<SensorEnum, FloatingType>(data, "air_speed") == FloatingType(7.2));
}
