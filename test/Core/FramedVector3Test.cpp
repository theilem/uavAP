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

	CHECK((Vector3)framedData == data);
}