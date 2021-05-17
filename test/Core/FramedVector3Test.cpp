//
// Created by seedship on 1/21/21.
//

#include <cpsCore/Utilities/Test/TestInfo.h>
#include "uavAP/Core/FramedVector3.h"

TEST_CASE("Constructor Test")
{
	Vector3 data = {1.1, 2.2, 3.3};
	FramedVector3 framedData = data;
	CHECK(framedData.x() == 1.1);
	CHECK(framedData.y() == 2.2);
	CHECK(framedData.z() == 3.3);
	CHECK(framedData.frame == Frame::INERTIAL);

	CHECK((Vector3) framedData == data);
}

TEST_CASE("Reference Test")
{
	Vector3 data = {1.1, 2.2, 3.3};
	FramedVector3 framedData = data;
	framedData.x() = 6.6;
	framedData.y() = 7.7;
	framedData.z() = 8.8;
	framedData.frame = Frame::BODY;
	CHECK(framedData.x() == 6.6);
	CHECK(framedData.y() == 7.7);
	CHECK(framedData.z() == 8.8);
	CHECK(framedData.frame == Frame::BODY);
}

void
vector3Modifier(Vector3& input)
{
	input[0] = 10.1;
	input[1] = 20.2;
	input[2] = 30.3;
}

TEST_CASE("Preserve Frame Test")
{
	Vector3 data = {1.1, 2.2, 3.3};
	FramedVector3 framedData = data;
	framedData.frame = Frame::BODY;
	vector3Modifier(framedData);

	CHECK(framedData.x() == 10.1);
	CHECK(framedData.y() == 20.2);
	CHECK(framedData.z() == 30.3);
	CHECK(framedData.frame == Frame::BODY);
}

TEST_CASE("Infer Inertial Test")
{
	Vector3 data = {1.1, 2.2, 3.3};
	FramedVector3 framedData = data;
	CHECK(framedData.frame == Frame::INERTIAL);
}