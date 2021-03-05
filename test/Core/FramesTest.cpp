/*
 * FramesTest.cpp
 *
 *  Created on: Aug 8, 2018
 *      Author: mircot
 */

#include <cpsCore/Utilities/Test/TestInfo.h>
#include <uavAP/Core/Frames/BodyFrame.h>
#include <uavAP/Core/Frames/InertialFrame.h>

TEST_CASE("Body Frame Test 1")
{
	Vector3 bodyRot(90, 0, 0);
	BodyFrame body(bodyRot * M_PI / 180.0);

	InertialFrame inert;

	Vector3 dirBody(1, 0, 0);

	CHECK(inert.fromFrameDirection(body, dirBody) == body.toInertialFrameDirection(dirBody));
	CHECK(inert.fromFrameDirection(body, dirBody) == dirBody);


	dirBody = Vector3(0, 1, 0);

	CHECK(inert.fromFrameDirection(body, dirBody) == body.toInertialFrameDirection(dirBody));
	CHECK(inert.fromFrameDirection(body, dirBody).x() == Approx(0).margin(1e-6));
	CHECK(inert.fromFrameDirection(body, dirBody).y() == Approx(0).margin(1e-6));
	CHECK(inert.fromFrameDirection(body, dirBody).z() == Approx(1).margin(1e-6));

}

//TEST_CASE("Body Frame Test 2")
//{
//	Vector3 bodyRot(45, 30, 90);
//	BodyFrame body(bodyRot * M_PI / 180.0);
//
//	InertialFrame inert;
//
//	Vector3 dirBody(1, 0, 0);
//
//	CHECK(inert.fromFrameDirection(body, dirBody) == body.toInertialFrameDirection(dirBody));
//	CHECK(inert.fromFrameDirection(body, dirBody) == dirBody);
//
//
//	dirBody = Vector3(0, 1, 0);
//
//	CHECK(inert.fromFrameDirection(body, dirBody) == body.toInertialFrameDirection(dirBody));
//	CHECK(inert.fromFrameDirection(body, dirBody).x()  == Approx(0).margin(1e-6));
//	CHECK(inert.fromFrameDirection(body, dirBody).y() == Approx(0).margin(1e-6));
//	CHECK(inert.fromFrameDirection(body, dirBody).z() == Approx(1).margin(1e-6));
//
//}
