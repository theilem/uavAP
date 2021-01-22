//
// Created by seedship on 1/21/21.
//

#include <cpsCore/Utilities/Test/TestInfo.h>
#include <iostream>
#include "uavAP/Core/FramedVector3.h"
#include "uavAP/Core/Orientation/ConversionUtils.h"

TEST_CASE("Test simpleFlipInternal")
{
	Vector3 a({1,2,3});
	simpleFlipInertial(a);
	Vector3 expected({2,1,-3});
	CHECK(a == expected);
}

// ENU

TEST_CASE("ENU inertial -> vehicle 1 conversion test 1"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::INERTIAL;

	Vector3 expected = {2, -1, 3};
	Vector3 attitude = {degToRad(12), degToRad(23), degToRad(90)};

	directionalConversion(velocity, attitude, Frame::VEHICLE_1, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::VEHICLE_1);
}

TEST_CASE("ENU vehicle 1 -> vehicle 2 conversion 1 test 1"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::VEHICLE_1;

	Vector3 expected = {1, 3, -2};
	Vector3 attitude = {degToRad(12), degToRad(90),degToRad(23)};

	directionalConversion(velocity, attitude, Frame::VEHICLE_2, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::VEHICLE_2);
}

TEST_CASE("ENU vehicle 2 -> Body conversion test 1"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::VEHICLE_2;

	Vector3 expected = {3, 2, -1};
	Vector3 attitude = {degToRad(90), degToRad(12),degToRad(23)};

	directionalConversion(velocity, attitude, Frame::BODY, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::BODY);
}

TEST_CASE("ENU inertial -> body conversion test 1"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::INERTIAL;

	Vector3 expected = {1, 3, -2};
	Vector3 attitude = {degToRad(90), degToRad(90), degToRad(90)};

	directionalConversion(velocity, attitude, Frame::BODY, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::BODY);
}

TEST_CASE("ENU inertial -> vehicle 1 conversion test 2"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::INERTIAL;

	Vector3 expected = {3.0/sqrt(2.0), 1.0/sqrt(2.0), 3.0};
	Vector3 attitude = {degToRad(12), degToRad(23), degToRad(45)};

	directionalConversion(velocity, attitude, Frame::VEHICLE_1, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::VEHICLE_1);
}

TEST_CASE("ENU vehicle 1 -> vehicle 2 conversion 1 test 2"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::VEHICLE_1;

	Vector3 expected = {1, 5.0/sqrt(2.0), 1.0/sqrt(2.0)};
	Vector3 attitude = {degToRad(12), degToRad(45),degToRad(23)};

	directionalConversion(velocity, attitude, Frame::VEHICLE_2, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::VEHICLE_2);
}

TEST_CASE("ENU vehicle 2 -> Body conversion test 2"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::VEHICLE_2;

	Vector3 expected = {2*sqrt(2), 2, sqrt(2)};
	Vector3 attitude = {degToRad(45), degToRad(12),degToRad(23)};

	directionalConversion(velocity, attitude, Frame::BODY, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::BODY);
}

TEST_CASE("ENU inertial -> body conversion test 2"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::INERTIAL;

	Vector3 expected = {3.0 - sqrt(2.0)/4.0, 1.0/2.0 + 3.0/sqrt(2.0), -sqrt(2.0)/4.0};
	Vector3 attitude = {degToRad(45), degToRad(45), degToRad(45)};

	directionalConversion(velocity, attitude, Frame::BODY, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::BODY);
}

TEST_CASE("ENU vehicle 1 -> inertial conversion test 1"){
	FramedVector3 velocity = Vector3({2, -1, 3});
	velocity.frame = Frame::VEHICLE_1;

	Vector3 expected = {1, 2, 3};
	Vector3 attitude = {degToRad(12), degToRad(23), degToRad(90)};

	directionalConversion(velocity, attitude, Frame::INERTIAL, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::INERTIAL);
}

TEST_CASE("ENU vehicle 2 -> vehicle 1 conversion test 1"){
	FramedVector3 velocity = Vector3({1, 3, -2});
	velocity.frame = Frame::VEHICLE_2;

	Vector3 expected = {1, 2, 3};
	Vector3 attitude = {degToRad(12), degToRad(90),degToRad(23)};

	directionalConversion(velocity, attitude, Frame::VEHICLE_1, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::VEHICLE_1);
}

TEST_CASE("ENU Body -> vehicle 2 conversion test 1"){
	FramedVector3 velocity = Vector3({3, 2, -1});
	velocity.frame = Frame::BODY;

	Vector3 expected = {1, 2, 3};
	Vector3 attitude = {degToRad(90), degToRad(12),degToRad(23)};

	directionalConversion(velocity, attitude, Frame::VEHICLE_2, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::VEHICLE_2);
}

TEST_CASE("ENU Body -> inertial conversion test 1"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::BODY;

	Vector3 expected = {1, -3, 2};
	Vector3 attitude = {degToRad(90), degToRad(90),degToRad(90)};

	directionalConversion(velocity, attitude, Frame::INERTIAL, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::INERTIAL);

}

TEST_CASE("ENU  vehicle 1 -> inertial conversion test 2"){
	FramedVector3 velocity = Vector3({3.0/sqrt(2.0), 1.0/sqrt(2.0), 3.0});
	velocity.frame = Frame::VEHICLE_1;

	Vector3 expected = {1, 2, 3};
	Vector3 attitude = {degToRad(12), degToRad(23), degToRad(45)};

	directionalConversion(velocity, attitude, Frame::INERTIAL, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::INERTIAL);
}

TEST_CASE("ENU vehicle 2 -> vehicle 1 conversion test 2"){
	FramedVector3 velocity = Vector3({1, 5.0/sqrt(2.0), 1.0/sqrt(2.0)});
	velocity.frame = Frame::VEHICLE_2;

	Vector3 expected = {1, 2, 3};
	Vector3 attitude = {degToRad(12), degToRad(45),degToRad(23)};

	directionalConversion(velocity, attitude, Frame::VEHICLE_1, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::VEHICLE_1);
}

TEST_CASE("ENU Body -> vehicle 2 conversion test 2"){
	FramedVector3 velocity = Vector3({2*sqrt(2), 2, sqrt(2)});
	velocity.frame = Frame::BODY;

	Vector3 expected = {1, 2, 3};
	Vector3 attitude = {degToRad(45), degToRad(12),degToRad(23)};

	directionalConversion(velocity, attitude, Frame::VEHICLE_2, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::VEHICLE_2);
}

TEST_CASE("ENU body -> inertial conversion test 2"){
	FramedVector3 velocity = Vector3({3.0 - sqrt(2.0)/4.0, 1.0/2.0 + 3.0/sqrt(2.0), -sqrt(2.0)/4.0});
	velocity.frame = Frame::BODY;

	Vector3 expected = {1, 2, 3};
	Vector3 attitude = {degToRad(45), degToRad(45), degToRad(45)};

	directionalConversion(velocity, attitude, Frame::INERTIAL, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::INERTIAL);
}

// NED

TEST_CASE("NED body -> vehicle 2 conversion test 1"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::BODY;

	Vector3 expected = {1, -3, 2};
	Vector3 attitude = {degToRad(90), degToRad(12), degToRad(23)};

	directionalConversion(velocity, attitude, Frame::VEHICLE_2, Orientation::NED);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::VEHICLE_2);
}

TEST_CASE("NED vehicle 2 -> vehicle 1 conversion test 1"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::VEHICLE_2;

	Vector3 expected = {3, 2, -1};
	Vector3 attitude = {degToRad(12), degToRad(90),degToRad(23)};

	directionalConversion(velocity, attitude, Frame::VEHICLE_1, Orientation::NED);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::VEHICLE_1);
}

TEST_CASE("NED vehicle 1 -> inertial conversion test 1"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::VEHICLE_1;

	Vector3 expected = {-2, 1, 3};
	Vector3 attitude = {degToRad(12),degToRad(23), degToRad(90)};

	directionalConversion(velocity, attitude, Frame::INERTIAL, Orientation::NED);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::INERTIAL);
}

TEST_CASE("NED body -> inertial conversion test 1"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::BODY;

	Vector3 expected = {3, 2, -1};
	Vector3 attitude = {degToRad(90), degToRad(90), degToRad(90)};

	directionalConversion(velocity, attitude, Frame::INERTIAL, Orientation::NED);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::INERTIAL);
}

TEST_CASE("NED vehicle 2 -> body conversion test 1"){
	FramedVector3 velocity = Vector3({1, -3, 2});
	velocity.frame = Frame::VEHICLE_2;

	Vector3 expected = {1, 2, 3};
	Vector3 attitude = {degToRad(90), degToRad(12), degToRad(23)};

	directionalConversion(velocity, attitude, Frame::BODY, Orientation::NED);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::BODY);
}

TEST_CASE("NED vehicle 1 -> vehicle 2 conversion test 1"){
	FramedVector3 velocity = Vector3({3, 2, -1});
	velocity.frame = Frame::VEHICLE_1;

	Vector3 expected = {1, 2, 3};
	Vector3 attitude = {degToRad(12), degToRad(90),degToRad(23)};

	directionalConversion(velocity, attitude, Frame::VEHICLE_2, Orientation::NED);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::VEHICLE_2);
}

TEST_CASE("NED inertial -> vehicle 1 conversion test 1"){
	FramedVector3 velocity = Vector3({-2, 1, 3});
	velocity.frame = Frame::INERTIAL;

	Vector3 expected = {1, 2, 3};
	Vector3 attitude = {degToRad(12),degToRad(23), degToRad(90)};

	directionalConversion(velocity, attitude, Frame::VEHICLE_1, Orientation::NED);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::VEHICLE_1);
}

TEST_CASE("NED inertial -> body conversion test 1"){
	FramedVector3 velocity = Vector3({3, 2, -1});
	velocity.frame = Frame::INERTIAL;

	Vector3 expected = {1, 2, 3};
	Vector3 attitude = {degToRad(90), degToRad(90), degToRad(90)};

	directionalConversion(velocity, attitude, Frame::BODY, Orientation::NED);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::BODY);
}

TEST_CASE("NED body -> vehicle 2 conversion test 2"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::BODY;

	Vector3 expected = {1, -1.0/sqrt(2.0), 5.0/sqrt(2)};
	Vector3 attitude = {degToRad(45), degToRad(12), degToRad(23)};

	directionalConversion(velocity, attitude, Frame::VEHICLE_2, Orientation::NED);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::VEHICLE_2);
}

TEST_CASE("NED vehicle 2 -> vehicle 1 conversion test 2`"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::VEHICLE_2;

	Vector3 expected = {2*sqrt(2.0), 2, sqrt(2.0)};
	Vector3 attitude = {degToRad(12), degToRad(45),degToRad(23)};

	directionalConversion(velocity, attitude, Frame::VEHICLE_1, Orientation::NED);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::VEHICLE_1);
}

TEST_CASE("NED vehicle 1 -> inertial conversion test 2"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::VEHICLE_1;

	Vector3 expected = {-1.0/sqrt(2), 3.0/sqrt(2), 3};
	Vector3 attitude = {degToRad(12),degToRad(23), degToRad(45)};

	directionalConversion(velocity, attitude, Frame::INERTIAL, Orientation::NED);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::INERTIAL);
}

TEST_CASE("NED body -> inertial conversion test 2"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::BODY;

	Vector3 expected = {1.0 + 5.0/2.0/sqrt(2), 5.0/2.0/sqrt(2), 5.0/2.0 - 1.0/sqrt(2.0)};
	Vector3 attitude = {degToRad(45), degToRad(45), degToRad(45)};

	directionalConversion(velocity, attitude, Frame::INERTIAL, Orientation::NED);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::INERTIAL);
}

TEST_CASE("NED vehicle 2 -> body conversion test 2"){
	FramedVector3 velocity = Vector3({1, -1.0/sqrt(2.0), 5.0/sqrt(2)});
	velocity.frame = Frame::VEHICLE_2;

	Vector3 expected = {1, 2, 3};
	Vector3 attitude = {degToRad(45), degToRad(12), degToRad(23)};

	directionalConversion(velocity, attitude, Frame::BODY, Orientation::NED);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::BODY);
}

TEST_CASE("NED vehicle 1 -> vehicle 2 conversion test 2`"){
	FramedVector3 velocity = Vector3({2*sqrt(2.0), 2, sqrt(2.0)});
	velocity.frame = Frame::VEHICLE_1;

	Vector3 expected = {1, 2, 3};
	Vector3 attitude = {degToRad(12), degToRad(45),degToRad(23)};

	directionalConversion(velocity, attitude, Frame::VEHICLE_2, Orientation::NED);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::VEHICLE_2);
}

TEST_CASE("NED inertial -> vehicle 1 conversion test 2"){
	FramedVector3 velocity = Vector3({-1.0/sqrt(2), 3.0/sqrt(2), 3});
	velocity.frame = Frame::INERTIAL;

	Vector3 expected = {1, 2, 3};
	Vector3 attitude = {degToRad(12),degToRad(23), degToRad(45)};

	directionalConversion(velocity, attitude, Frame::VEHICLE_1, Orientation::NED);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::VEHICLE_1);
}

TEST_CASE("NED inertial -> body conversion test 2"){
	FramedVector3 velocity = Vector3({1.0 + 5.0/2.0/sqrt(2), 5.0/2.0/sqrt(2), 5.0/2.0 - 1.0/sqrt(2.0)});
	velocity.frame = Frame::INERTIAL;

	Vector3 expected = {1, 2, 3};
	Vector3 attitude = {degToRad(45), degToRad(45), degToRad(45)};

	directionalConversion(velocity, attitude, Frame::BODY, Orientation::NED);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::BODY);
}