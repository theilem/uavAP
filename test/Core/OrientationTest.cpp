//
// Created by seedship on 1/21/21.
//

#include <cpsCore/Utilities/Test/TestInfo.h>
#include <uavAP/Core/SensorData.h>
#include "uavAP/Core/Orientation/ConversionUtils.h"


TEST_CASE("ENU inertial -> vehicle 1 frame conversion"){
	SensorData sd;
	sd.velocity = Vector3({1, 2, 3});
	sd.velocity.frame = Frame::INERTIAL;

	Vector3 expected = {-2, 1, 3};
	sd.attitude = {0, 0, degToRad(90)};

	directionalToFrameENU(sd.velocity, sd.attitude, Frame::VEHICLE_1);
	CHECK(sd.velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(sd.velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(sd.velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(sd.velocity.frame == Frame::VEHICLE_1);
}