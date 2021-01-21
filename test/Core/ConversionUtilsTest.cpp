//
// Created by seedship on 1/21/21.
//

#include <cpsCore/Utilities/Test/TestInfo.h>
#include "uavAP/Core/FramedVector3.h"
#include "uavAP/Core/Orientation/ConversionUtils.h"

TEST_CASE("Test simpleFlipInternal")
{
	Vector3 a({1,2,3});
	simpleFlipInertial(a);
	Vector3 expected({2,1,-3});
	CHECK(a == expected);
}

TEST_CASE("ENU inertial -> vehicle 1 conversion"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::INERTIAL;

	Vector3 expected = {-2, 1, 3};
	Vector3 attitude = {degToRad(12), degToRad(23), degToRad(90)};

	directionalToFrameENU(velocity, attitude, Frame::VEHICLE_1);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::VEHICLE_1);
}

TEST_CASE("ENU vehicle 1 -> vehicle 2 conversion"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::VEHICLE_1;

	Vector3 expected = {1, 3, -2};
	Vector3 attitude = {degToRad(90), degToRad(12),degToRad(23)};

	directionalToFrameENU(velocity, attitude, Frame::VEHICLE_2);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::VEHICLE_2);
}