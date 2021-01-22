//
// Created by seedship on 1/21/21.
//

#include <cpsCore/Utilities/Test/TestInfo.h>
#include <uavAP/Core/SensorData.h>
#include <uavAP/Core/Orientation/ENU.h>
#include <uavAP/Core/Orientation/NED.h>



TEST_CASE("NED conversion test")
{
	SensorData sd;
	sd.orientation = Orientation::ENU;
	NED::convert(sd);
	assert(sd.orientation == Orientation::NED);
}

TEST_CASE("ENU conversion test")
{
	SensorData sd;
	sd.orientation = Orientation::NED;
	ENU::convert(sd);
	assert(sd.orientation == Orientation::ENU);
}
