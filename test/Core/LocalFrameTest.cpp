//
// Created by seedship on 1/23/21.
//

#include <cpsCore/Utilities/Test/TestInfo.h>
#include "uavAP/Core/Frames/LocalFrame.h"
#include "uavAP/Core/SensorData.h"

TEST_CASE("To Local Frame Test 1")
{
	SensorData sd;

	LocalFrame frame({10, 20, 30}, degToRad(90));
	sd.position = {11, 22, 33};
	sd.attitude[2] = degToRad(46.0);
	sd.isInLocalFrame = false;

	frame.toLocalFrame(sd);

	Vector3 expected = {2, -1, 3};
	FloatingType expectedYaw = degToRad(46.0 - 90.0);

	CHECK(sd.position.x() == Approx(expected.x()).margin(1e-6));
	CHECK(sd.position.y() == Approx(expected.y()).margin(1e-6));
	CHECK(sd.position.z() == Approx(expected.z()).margin(1e-6));
	CHECK(sd.attitude[2] == Approx(expectedYaw).margin(1e-6));
	CHECK(sd.isInLocalFrame);
}

TEST_CASE("To Local Frame Test 2")
{
	SensorData sd;

	LocalFrame frame({10, 20, 30}, degToRad(45));
	sd.position = {11, 22, 33};
	sd.attitude[2] = degToRad(46.0);
	sd.isInLocalFrame = false;

	frame.toLocalFrame(sd);

	Vector3 expected = {3.0/sqrt(2.0), 1.0/sqrt(2.0), 3.0};
	FloatingType expectedYaw = degToRad(46.0 - 45.0);

	CHECK(sd.position.x() == Approx(expected.x()).margin(1e-6));
	CHECK(sd.position.y() == Approx(expected.y()).margin(1e-6));
	CHECK(sd.position.z() == Approx(expected.z()).margin(1e-6));
	CHECK(sd.attitude[2] == Approx(expectedYaw).margin(1e-6));
	CHECK(sd.isInLocalFrame);
}



TEST_CASE("Local Idempotency test")
{
	SensorData sd;

	LocalFrame frame({10, 20, 30}, degToRad(45));
	sd.position = {11, 22, 33};
	sd.attitude[2] = degToRad(46.0);
	sd.isInLocalFrame = true;

	frame.toLocalFrame(sd);

	Vector3 expected = {11, 22, 33};
	FloatingType expectedYaw = degToRad(46.0);

	CHECK(sd.position.x() == Approx(expected.x()).margin(1e-6));
	CHECK(sd.position.y() == Approx(expected.y()).margin(1e-6));
	CHECK(sd.position.z() == Approx(expected.z()).margin(1e-6));
	CHECK(sd.attitude[2] == Approx(expectedYaw).margin(1e-6));
	CHECK(sd.isInLocalFrame);
}

TEST_CASE("To Global Frame Test 1")
{
	SensorData sd;

	LocalFrame frame({10, 20, 30}, degToRad(90));
	sd.position = {2, -1, 3};
	sd.attitude[2] = degToRad(46.0 - 90.0);
	sd.isInLocalFrame = true;

	frame.toGlobalFrame(sd);

	Vector3 expected = {11, 22, 33};
	FloatingType expectedYaw = degToRad(46.0);

	CHECK(sd.position.x() == Approx(expected.x()).margin(1e-6));
	CHECK(sd.position.y() == Approx(expected.y()).margin(1e-6));
	CHECK(sd.position.z() == Approx(expected.z()).margin(1e-6));
	CHECK(sd.attitude[2] == Approx(expectedYaw).margin(1e-6));
	CHECK(!sd.isInLocalFrame);
}

TEST_CASE("To Global Frame Test 2")
{
	SensorData sd;

	LocalFrame frame({10, 20, 30}, degToRad(45));
	sd.position = {3.0/sqrt(2.0), 1.0/sqrt(2.0), 3.0};
	sd.attitude[2] = degToRad(46.0 - 45.0);
	sd.isInLocalFrame = true;

	frame.toGlobalFrame(sd);

	Vector3 expected = {11, 22, 33};
	FloatingType expectedYaw = degToRad(46.0);

	CHECK(sd.position.x() == Approx(expected.x()).margin(1e-6));
	CHECK(sd.position.y() == Approx(expected.y()).margin(1e-6));
	CHECK(sd.position.z() == Approx(expected.z()).margin(1e-6));
	CHECK(sd.attitude[2] == Approx(expectedYaw).margin(1e-6));
	CHECK(!sd.isInLocalFrame);
}



TEST_CASE("Idempotency test")
{
	SensorData sd;

	LocalFrame frame({10, 20, 30}, degToRad(45));
	sd.position = {11, 22, 33};
	sd.attitude[2] = degToRad(46.0);
	sd.isInLocalFrame = false;

	frame.toGlobalFrame(sd);

	Vector3 expected = {11, 22, 33};
	FloatingType expectedYaw = degToRad(46.0);

	CHECK(sd.position.x() == Approx(expected.x()).margin(1e-6));
	CHECK(sd.position.y() == Approx(expected.y()).margin(1e-6));
	CHECK(sd.position.z() == Approx(expected.z()).margin(1e-6));
	CHECK(sd.attitude[2] == Approx(expectedYaw).margin(1e-6));
	CHECK(!sd.isInLocalFrame);
}