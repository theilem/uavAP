//
// Created by seedship on 1/21/21.
//

#include <cpsCore/Utilities/Test/TestInfo.h>
#include <iostream>
#include "uavAP/Core/FramedVector3.h"
#include "uavAP/Core/Orientation/ConversionUtils.h"

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

	Vector3 expected = {-3, 2, 1};
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

	Vector3 expected = {-1, 3, 2};
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

	Vector3 expected = {-sqrt(2), 2, 2*sqrt(2)};
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

	Vector3 expected = {sqrt(2.0)/4.0, 1.0/2.0 + 3.0/sqrt(2.0), 3 - sqrt(2.0)/4.0};
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
	FramedVector3 velocity = Vector3({-3, 2, 1});
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

	Vector3 expected = {-1, 3, 2};
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
	FramedVector3 velocity = Vector3({-sqrt(2), 2, 2 *sqrt(2)});
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
	FramedVector3 velocity = Vector3({sqrt(2.0)/4.0, 1.0/2.0 + 3.0/sqrt(2.0), 3 - sqrt(2.0)/4.0});
	velocity.frame = Frame::BODY;

	Vector3 expected = {1, 2, 3};
	Vector3 attitude = {degToRad(45), degToRad(45), degToRad(45)};

	directionalConversion(velocity, attitude, Frame::INERTIAL, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::INERTIAL);
}

TEST_CASE("ENU inertial -> body test")
{
	FloatingType roll = degToRad(15);
	FloatingType pitch = degToRad(35);
	FloatingType yaw = degToRad(55);
	FloatingType rollRate = degToRad(20);
	FloatingType pitchRate = degToRad(40);
	FloatingType yawRate = degToRad(60);
	Vector3 attitude({roll, pitch, yaw});
	FramedVector3 angularRate = Vector3({rollRate, pitchRate, yawRate});
	FloatingType p = pitchRate * cos(roll) - yawRate * sin(roll) * cos(pitch);
	FloatingType q = yawRate * sin(pitch) + rollRate;
	FloatingType r = yawRate * cos(roll) * cos(pitch) + pitchRate * sin(roll);


	CHECK(angularRate.frame == Frame::INERTIAL);
	angularConversion(angularRate, attitude, Frame::BODY, Orientation::ENU);

	CHECK(angularRate[0] == Approx(p).margin(1e-6));
	CHECK(angularRate[1] == Approx(q).margin(1e-6));
	CHECK(angularRate[2] == Approx(r).margin(1e-6));
	CHECK(angularRate.frame == Frame::BODY);
}

TEST_CASE("ENU body -> inertial test")
{
	FloatingType roll = degToRad(15);
	FloatingType pitch = degToRad(35);
	FloatingType yaw = degToRad(55);
	Vector3 attitude({roll, pitch, yaw});
	FloatingType rollRate = degToRad(20);
	FloatingType pitchRate = degToRad(40);
	FloatingType yawRate = degToRad(60);
	FloatingType p = pitchRate * cos(roll) - yawRate * sin(roll) * cos(pitch);
	FloatingType q = yawRate * sin(pitch) + rollRate;
	FloatingType r = yawRate * cos(roll) * cos(pitch) + pitchRate * sin(roll);
	FramedVector3 angularRate = Vector3({p, q, r});
	angularRate.frame = Frame::BODY;

	CHECK(angularRate.frame == Frame::BODY);
	angularConversion(angularRate, attitude, Frame::INERTIAL, Orientation::ENU);

	CHECK(angularRate[0] == Approx(rollRate).margin(1e-6));
	CHECK(angularRate[1] == Approx(pitchRate).margin(1e-6));
	CHECK(angularRate[2] == Approx(yawRate).margin(1e-6));
	CHECK(angularRate.frame == Frame::INERTIAL);
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

TEST_CASE("NED inertial -> body test")
{
	FloatingType roll = degToRad(15);
	FloatingType pitch = degToRad(35);
	FloatingType yaw = degToRad(55);
	FloatingType rollRate = degToRad(20);
	FloatingType pitchRate = degToRad(40);
	FloatingType yawRate = degToRad(60);
	Vector3 attitude({roll, pitch, yaw});
	FramedVector3 angularRate = Vector3({rollRate, pitchRate, yawRate});
	FloatingType p = rollRate - yawRate * sin(pitch);
	FloatingType q = pitchRate * cos(roll) + yawRate * cos(pitch) * sin(roll);
	FloatingType r = yawRate * cos(pitch) * cos(roll) - pitchRate * sin(roll);


	CHECK(angularRate.frame == Frame::INERTIAL);
	angularConversion(angularRate, attitude, Frame::BODY, Orientation::NED);

	CHECK(angularRate[0] == Approx(p).margin(1e-6));
	CHECK(angularRate[1] == Approx(q).margin(1e-6));
	CHECK(angularRate[2] == Approx(r).margin(1e-6));
	CHECK(angularRate.frame == Frame::BODY);
}

TEST_CASE("NED body -> inertial test")
{
	FloatingType roll = degToRad(15);
	FloatingType pitch = degToRad(35);
	FloatingType yaw = degToRad(55);
	Vector3 attitude({roll, pitch, yaw});
	FloatingType rollRate = degToRad(20);
	FloatingType pitchRate = degToRad(40);
	FloatingType yawRate = degToRad(60);
	FloatingType p = rollRate - yawRate * sin(pitch);
	FloatingType q = pitchRate * cos(roll) + yawRate * cos(pitch) * sin(roll);
	FloatingType r = yawRate * cos(pitch) * cos(roll) - pitchRate * sin(roll);
	FramedVector3 angularRate = Vector3({p, q, r});
	angularRate.frame = Frame::BODY;

	CHECK(angularRate.frame == Frame::BODY);
	angularConversion(angularRate, attitude, Frame::INERTIAL, Orientation::NED);

	CHECK(angularRate[0] == Approx(rollRate).margin(1e-6));
	CHECK(angularRate[1] == Approx(pitchRate).margin(1e-6));
	CHECK(angularRate[2] == Approx(yawRate).margin(1e-6));
	CHECK(angularRate.frame == Frame::INERTIAL);
}