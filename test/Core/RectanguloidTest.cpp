//
// Created by mirco on 25.02.21.
//

#include <cpsCore/Utilities/Test/TestInfo.h>
#include <uavAP/Core/Rectanguloid.h>

TEST_CASE("Rectanguloid is inside")
{
	Rectanguloid rect;

	rect.center = Vector3(100, 0, 200);
	rect.majorSideLength = 100;
	rect.minorSideLength = 50;
	rect.majorSideOrientation = 0;
	rect.height = 100;

	CHECK(rect.isInside(Vector3(100, 0, 200)));
	CHECK(rect.isInside(Vector3(100, 0, 150)));
	CHECK_FALSE(rect.isInside(Vector3(100, 0, 149)));
	CHECK_FALSE(rect.isInside(Vector3(100, 0, 251)));

	CHECK(rect.isInside(Vector3(150, 0, 200)));
	CHECK(rect.isInside(Vector3(150, 25, 200)));
	CHECK(rect.isInside(Vector3(150, -25, 200)));
	CHECK(rect.isInside(Vector3(50, -25, 200)));
	CHECK_FALSE(rect.isInside(Vector3(49, -25, 200)));
	CHECK_FALSE(rect.isInside(Vector3(50, -26, 200)));


	rect.majorSideOrientation = degToRad(90);
	CHECK(rect.isInside(Vector3(100, -50, 200)));
	CHECK_FALSE(rect.isInside(Vector3(100, -51, 200)));

}