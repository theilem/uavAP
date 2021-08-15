//
// Created by seedship on 1/21/21.
//

#include <cpsCore/Utilities/Test/TestInfo.h>
#include <iostream>
#include "uavAP/Core/FramedVector3.h"
#include "uavAP/Core/Orientation/ConversionUtils.h"
#include "uavAP/Core/Orientation/ENU.h"
#include "uavAP/Core/Orientation/NED.h"

// ENU

TEST_CASE("ENU inertial -> vehicle 1 conversion test 1"){
	FramedVector3 velocity = Vector3({1, 2, 3});
	velocity.frame = Frame::INERTIAL;

	Vector3 expected = {1, 2, 3};
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

	Vector3 expected = {2, 3, 1};
	Vector3 attitude = {degToRad(90), degToRad(90), degToRad(90)};

	directionalConversion(velocity, attitude, Frame::BODY, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::BODY);
}

// NOTE: forgot to account for 90 degree shift in Z rotation when doing ENU. TODO fix test case
//TEST_CASE("ENU inertial -> vehicle 1 conversion test 2"){
//	FramedVector3 velocity = Vector3({1, 2, 3});
//	velocity.frame = Frame::INERTIAL;
//
//	Vector3 expected = {3.0/sqrt(2.0), 1.0/sqrt(2.0), 3.0};
//	Vector3 attitude = {degToRad(12), degToRad(23), degToRad(45)};
//
//	directionalConversion(velocity, attitude, Frame::VEHICLE_1, Orientation::ENU);
//	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
//	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
//	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
//	CHECK(velocity.frame == Frame::VEHICLE_1);
//}

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


// NOTE: forgot to account for 90 degree shift in Z rotation when doing ENU. TODO fix test case
//TEST_CASE("ENU inertial -> body conversion test 2"){
//	FramedVector3 velocity = Vector3({1, 2, 3});
//	velocity.frame = Frame::INERTIAL;
//
//	Vector3 expected = {sqrt(2.0)/4.0, 1.0/2.0 + 3.0/sqrt(2.0), 3 - sqrt(2.0)/4.0};
//	Vector3 attitude = {degToRad(45), degToRad(45), degToRad(45)};
//
//	directionalConversion(velocity, attitude, Frame::BODY, Orientation::ENU);
//	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
//	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
//	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
//	CHECK(velocity.frame == Frame::BODY);
//}

TEST_CASE("ENU vehicle 1 -> inertial conversion test 1"){
	FramedVector3 velocity = Vector3({2, -1, 3});
	velocity.frame = Frame::VEHICLE_1;

	Vector3 expected = {2, -1, 3};
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

	Vector3 expected = {3, 1, 2};
	Vector3 attitude = {degToRad(90), degToRad(90),degToRad(90)};

	directionalConversion(velocity, attitude, Frame::INERTIAL, Orientation::ENU);
	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
	CHECK(velocity.frame == Frame::INERTIAL);

}

// NOTE: forgot to account for 90 degree shift in Z rotation when doing ENU. TODO fix test case
//TEST_CASE("ENU  vehicle 1 -> inertial conversion test 2"){
//	FramedVector3 velocity = Vector3({3.0/sqrt(2.0), 1.0/sqrt(2.0), 3.0});
//	velocity.frame = Frame::VEHICLE_1;
//
//	Vector3 expected = {1, 2, 3};
//	Vector3 attitude = {degToRad(12), degToRad(23), degToRad(45)};
//
//	directionalConversion(velocity, attitude, Frame::INERTIAL, Orientation::ENU);
//	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
//	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
//	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
//	CHECK(velocity.frame == Frame::INERTIAL);
//}

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

// NOTE: forgot to account for 90 degree shift in Z rotation when doing ENU. TODO fix test case
//TEST_CASE("ENU body -> inertial conversion test 2"){
//	FramedVector3 velocity = Vector3({sqrt(2.0)/4.0, 1.0/2.0 + 3.0/sqrt(2.0), 3 - sqrt(2.0)/4.0});
//	velocity.frame = Frame::BODY;
//
//	Vector3 expected = {1, 2, 3};
//	Vector3 attitude = {degToRad(45), degToRad(45), degToRad(45)};
//
//	directionalConversion(velocity, attitude, Frame::INERTIAL, Orientation::ENU);
//	CHECK(velocity.x() == Approx(expected.x()).margin(1e-6));
//	CHECK(velocity.y() == Approx(expected.y()).margin(1e-6));
//	CHECK(velocity.z() == Approx(expected.z()).margin(1e-6));
//	CHECK(velocity.frame == Frame::INERTIAL);
//}

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

TEST_CASE("ENU Body -> Inertial -> Body test direction")
{
	FloatingType roll = degToRad(15);
	FloatingType pitch = degToRad(35);
	FloatingType yaw = degToRad(55);
	Vector3 attitude({roll, pitch, yaw});
	FloatingType x = 2;
	FloatingType y = 3;
	FloatingType z = 4;

	FramedVector3 vec = Vector3({x, y, z});
	vec.frame = Frame::BODY;

	directionalConversion(vec, attitude, Frame::INERTIAL, Orientation::ENU);
	// TODO maybe calculate expected values here later
	CHECK(vec.x() != 2);
	CHECK(vec.y() != 3);
	CHECK(vec.z() != 4);
	directionalConversion(vec, attitude, Frame::BODY, Orientation::ENU);
	CHECK(vec.x() == Approx(2).margin(1e-6));
	CHECK(vec.y() == Approx(3).margin(1e-6));
	CHECK(vec.z() == Approx(4).margin(1e-6));
}

TEST_CASE("ENU Body -> Inertial -> Body test angular")
{
	FloatingType roll = degToRad(15);
	FloatingType pitch = degToRad(35);
	FloatingType yaw = degToRad(55);
	Vector3 attitude({roll, pitch, yaw});

	FramedVector3 vec = Vector3({0.1, 0.2, 0.3});
	vec.frame = Frame::BODY;

	angularConversion(vec, attitude, Frame::INERTIAL, Orientation::ENU);
	// TODO maybe calculate expected values here later
	CHECK(vec.x() != 0.1);
	CHECK(vec.y() != 0.2);
	CHECK(vec.z() != 0.3);
	angularConversion(vec, attitude, Frame::BODY, Orientation::ENU);
	CHECK(vec.x() == Approx(0.1).margin(1e-6));
	CHECK(vec.y() == Approx(0.2).margin(1e-6));
	CHECK(vec.z() == Approx(0.3).margin(1e-6));
}

TEST_CASE("ENU Inertial -> Body -> Inertial test direction")
{
	FloatingType roll = degToRad(15);
	FloatingType pitch = degToRad(35);
	FloatingType yaw = degToRad(55);
	Vector3 attitude({roll, pitch, yaw});
	FloatingType x = 2;
	FloatingType y = 3;
	FloatingType z = 4;

	FramedVector3 accl = Vector3({x, y, z});
	accl.frame = Frame::INERTIAL;

	directionalConversion(accl, attitude, Frame::BODY, Orientation::ENU);
	// TODO maybe calculate expected values here later
	CHECK(accl.x() != 2);
	CHECK(accl.y() != 3);
	CHECK(accl.z() != 4);
	directionalConversion(accl, attitude, Frame::INERTIAL, Orientation::ENU);
	CHECK(accl.x() == Approx(2).margin(1e-6));
	CHECK(accl.y() == Approx(3).margin(1e-6));
	CHECK(accl.z() == Approx(4).margin(1e-6));
}

TEST_CASE("ENU Inertial -> Body -> Inertial test angular")
{
	FloatingType roll = degToRad(15);
	FloatingType pitch = degToRad(35);
	FloatingType yaw = degToRad(55);
	Vector3 attitude({roll, pitch, yaw});
	FloatingType x = 0.1;
	FloatingType y = 0.2;
	FloatingType z = 0.3;

	FramedVector3 accl = Vector3({x, y, z});
	accl.frame = Frame::INERTIAL;

	angularConversion(accl, attitude, Frame::BODY, Orientation::ENU);
	// TODO maybe calculate expected values here later
	CHECK(accl.x() != 0.1);
	CHECK(accl.y() != 0.2);
	CHECK(accl.z() != 0.3);
	angularConversion(accl, attitude, Frame::INERTIAL, Orientation::ENU);
	CHECK(accl.x() == Approx(0.1).margin(1e-6));
	CHECK(accl.y() == Approx(0.2).margin(1e-6));
	CHECK(accl.z() == Approx(0.3).margin(1e-6));
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

TEST_CASE("NED Body -> Inertial -> Body test direction")
{
	FloatingType roll = degToRad(15);
	FloatingType pitch = degToRad(35);
	FloatingType yaw = degToRad(55);
	Vector3 attitude({roll, pitch, yaw});
	FloatingType x = 2;
	FloatingType y = 3;
	FloatingType z = 4;

	FramedVector3 accl = Vector3({x, y, z});
	accl.frame = Frame::BODY;

	directionalConversion(accl, attitude, Frame::INERTIAL, Orientation::NED);
	// TODO maybe calculate expected values here later
	CHECK(accl.x() != 2);
	CHECK(accl.y() != 3);
	CHECK(accl.z() != 4);
	directionalConversion(accl, attitude, Frame::BODY, Orientation::NED);
	CHECK(accl.x() == Approx(2).margin(1e-6));
	CHECK(accl.y() == Approx(3).margin(1e-6));
	CHECK(accl.z() == Approx(4).margin(1e-6));
}

TEST_CASE("NED Body -> Inertial -> Body test angular")
{
	FloatingType roll = degToRad(15);
	FloatingType pitch = degToRad(35);
	FloatingType yaw = degToRad(55);
	Vector3 attitude({roll, pitch, yaw});
	FloatingType x = 0.1;
	FloatingType y = 0.2;
	FloatingType z = 0.3;

	FramedVector3 accl = Vector3({x, y, z});
	accl.frame = Frame::BODY;

	angularConversion(accl, attitude, Frame::INERTIAL, Orientation::NED);
	// TODO maybe calculate expected values here later
	CHECK(accl.x() != 0.1);
	CHECK(accl.y() != 0.2);
	CHECK(accl.z() != 0.3);
	angularConversion(accl, attitude, Frame::BODY, Orientation::NED);
	CHECK(accl.x() == Approx(0.1).margin(1e-6));
	CHECK(accl.y() == Approx(0.2).margin(1e-6));
	CHECK(accl.z() == Approx(0.3).margin(1e-6));
}

TEST_CASE("NED Inertial -> Body -> Inertial test direction")
{
	FloatingType roll = degToRad(15);
	FloatingType pitch = degToRad(35);
	FloatingType yaw = degToRad(55);
	Vector3 attitude({roll, pitch, yaw});
	FloatingType x = 2;
	FloatingType y = 3;
	FloatingType z = 4;

	FramedVector3 accl = Vector3({x, y, z});
	accl.frame = Frame::INERTIAL;

	directionalConversion(accl, attitude, Frame::BODY, Orientation::NED);
	// TODO maybe calculate expected values here later
	CHECK(accl.x() != 2);
	CHECK(accl.y() != 3);
	CHECK(accl.z() != 4);
	directionalConversion(accl, attitude, Frame::INERTIAL, Orientation::NED);
	CHECK(accl.x() == Approx(2).margin(1e-6));
	CHECK(accl.y() == Approx(3).margin(1e-6));
	CHECK(accl.z() == Approx(4).margin(1e-6));
}

TEST_CASE("NED Inertial -> Body -> Inertial test angular")
{
	FloatingType roll = degToRad(15);
	FloatingType pitch = degToRad(35);
	FloatingType yaw = degToRad(55);
	Vector3 attitude({roll, pitch, yaw});
	FloatingType x = 0.1;
	FloatingType y = 0.2;
	FloatingType z = 0.3;

	FramedVector3 accl = Vector3({x, y, z});
	accl.frame = Frame::INERTIAL;

	angularConversion(accl, attitude, Frame::BODY, Orientation::NED);
	// TODO maybe calculate expected values here later
	CHECK(accl.x() != 0.1);
	CHECK(accl.y() != 0.2);
	CHECK(accl.z() != 0.3);
	angularConversion(accl, attitude, Frame::INERTIAL, Orientation::NED);
	CHECK(accl.x() == Approx(0.1).margin(1e-6));
	CHECK(accl.y() == Approx(0.2).margin(1e-6));
	CHECK(accl.z() == Approx(0.3).margin(1e-6));
}

// Orientation Conversions

TEST_CASE("ENU -> NED -> ENU Inertial")
{
	SensorData sd;
	sd.orientation = Orientation::ENU;
	sd.position = {100, 200, 300};

	sd.velocity = Vector3({2, 3, 4});
	sd.velocity.frame = Frame::INERTIAL;

	sd.acceleration = Vector3({1, 2, 3});
	sd.acceleration.frame = Frame::INERTIAL;

	sd.attitude = Vector3({0.1, 0.2, 0.3});

	sd.angularRate = Vector3({0.2, 0.3, 0.4});
	sd.angularRate.frame = Frame::INERTIAL;
	sd.groundSpeed = sd.velocity.norm();
	sd.airSpeed = sd.groundSpeed + 0.3;

	// NOTE not calculaing AoA, but maybe we should
	NED::convert(sd, Frame::INERTIAL, Frame::INERTIAL, Frame::INERTIAL);
	CHECK(sd.position.x() == Approx(200).margin(1e-6));
	CHECK(sd.position.y() == Approx(100).margin(1e-6));
	CHECK(sd.position.z() == Approx(-300).margin(1e-6));

	CHECK(sd.velocity.x() == Approx(3).margin(1e-6));
	CHECK(sd.velocity.y() == Approx(2).margin(1e-6));
	CHECK(sd.velocity.z() == Approx(-4).margin(1e-6));
	CHECK(sd.velocity.frame == Frame::INERTIAL);

	CHECK(sd.acceleration.x() == Approx(2).margin(1e-6));
	CHECK(sd.acceleration.y() == Approx(1).margin(1e-6));
	CHECK(sd.acceleration.z() == Approx(-3).margin(1e-6));
	CHECK(sd.acceleration.frame == Frame::INERTIAL);

	CHECK(sd.attitude.x() == Approx(0.1).margin(1e-6));
	CHECK(sd.attitude.y() == Approx(0.2).margin(1e-6));
	CHECK(sd.attitude.z() == Approx(-0.3 + degToRad(90)).margin(1e-6));

	CHECK(sd.angularRate.x() == Approx(0.2).margin(1e-6));
	CHECK(sd.angularRate.y() == Approx(0.3).margin(1e-6));
	CHECK(sd.angularRate.z() == Approx(-0.4).margin(1e-6));
	CHECK(sd.angularRate.frame == Frame::INERTIAL);

	CHECK(sd.groundSpeed == Approx(sqrt(29)).margin(1e-6));
	CHECK(sd.airSpeed == Approx(sqrt(29) + 0.3).margin(1e-6));

	ENU::convert(sd, Frame::INERTIAL, Frame::INERTIAL, Frame::INERTIAL);
	CHECK(sd.position.x() == Approx(100).margin(1e-6));
	CHECK(sd.position.y() == Approx(200).margin(1e-6));
	CHECK(sd.position.z() == Approx(300).margin(1e-6));

	CHECK(sd.velocity.x() == Approx(2).margin(1e-6));
	CHECK(sd.velocity.y() == Approx(3).margin(1e-6));
	CHECK(sd.velocity.z() == Approx(4).margin(1e-6));
	CHECK(sd.velocity.frame == Frame::INERTIAL);

	CHECK(sd.acceleration.x() == Approx(1).margin(1e-6));
	CHECK(sd.acceleration.y() == Approx(2).margin(1e-6));
	CHECK(sd.acceleration.z() == Approx(3).margin(1e-6));
	CHECK(sd.acceleration.frame == Frame::INERTIAL);

	CHECK(sd.attitude.x() == Approx(0.1).margin(1e-6));
	CHECK(sd.attitude.y() == Approx(0.2).margin(1e-6));
	CHECK(sd.attitude.z() == Approx(0.3).margin(1e-6));

	CHECK(sd.angularRate.x() == Approx(0.2).margin(1e-6));
	CHECK(sd.angularRate.y() == Approx(0.3).margin(1e-6));
	CHECK(sd.angularRate.z() == Approx(0.4).margin(1e-6));
	CHECK(sd.angularRate.frame == Frame::INERTIAL);

	CHECK(sd.groundSpeed == Approx(sqrt(29)).margin(1e-6));
	CHECK(sd.airSpeed == Approx(sqrt(29) + 0.3).margin(1e-6));
}

TEST_CASE("ENU -> NED -> ENU Body")
{
	SensorData sd;
	sd.orientation = Orientation::ENU;
	sd.position = {100, 200, 300};

	sd.velocity = Vector3({2, 3, 4});
	sd.velocity.frame = Frame::BODY;

	sd.acceleration = Vector3({1, 2, 3});
	sd.acceleration.frame = Frame::BODY;

	sd.attitude = Vector3({0.1, 0.2, 0.3});

	sd.angularRate = Vector3({0.2, 0.3, 0.4});
	sd.angularRate.frame = Frame::BODY;
	sd.groundSpeed = sd.velocity.norm();
	sd.airSpeed = sd.groundSpeed + 0.3;

	// NOTE not calculaing AoA, but maybe we should
	NED::convert(sd, Frame::BODY, Frame::BODY, Frame::BODY);
	CHECK(sd.position.x() == Approx(200).margin(1e-6));
	CHECK(sd.position.y() == Approx(100).margin(1e-6));
	CHECK(sd.position.z() == Approx(-300).margin(1e-6));

	CHECK(sd.velocity.x() == Approx(3).margin(1e-6));
	CHECK(sd.velocity.y() == Approx(2).margin(1e-6));
	CHECK(sd.velocity.z() == Approx(-4).margin(1e-6));
	CHECK(sd.velocity.frame == Frame::BODY);

	CHECK(sd.acceleration.x() == Approx(2).margin(1e-6));
	CHECK(sd.acceleration.y() == Approx(1).margin(1e-6));
	CHECK(sd.acceleration.z() == Approx(-3).margin(1e-6));
	CHECK(sd.acceleration.frame == Frame::BODY);

	CHECK(sd.attitude.x() == Approx(0.1).margin(1e-6));
	CHECK(sd.attitude.y() == Approx(0.2).margin(1e-6));
	CHECK(sd.attitude.z() == Approx(-0.3 + degToRad(90)).margin(1e-6));

	CHECK(sd.angularRate.x() == Approx(0.3).margin(1e-6));
	CHECK(sd.angularRate.y() == Approx(0.2).margin(1e-6));
	CHECK(sd.angularRate.z() == Approx(-0.4).margin(1e-6));
	CHECK(sd.angularRate.frame == Frame::BODY);

	CHECK(sd.groundSpeed == Approx(sqrt(29)).margin(1e-6));
	CHECK(sd.airSpeed == Approx(sqrt(29) + 0.3).margin(1e-6));

	ENU::convert(sd, Frame::BODY, Frame::BODY, Frame::BODY);
	CHECK(sd.position.x() == Approx(100).margin(1e-6));
	CHECK(sd.position.y() == Approx(200).margin(1e-6));
	CHECK(sd.position.z() == Approx(300).margin(1e-6));

	CHECK(sd.velocity.x() == Approx(2).margin(1e-6));
	CHECK(sd.velocity.y() == Approx(3).margin(1e-6));
	CHECK(sd.velocity.z() == Approx(4).margin(1e-6));
	CHECK(sd.velocity.frame == Frame::BODY);

	CHECK(sd.acceleration.x() == Approx(1).margin(1e-6));
	CHECK(sd.acceleration.y() == Approx(2).margin(1e-6));
	CHECK(sd.acceleration.z() == Approx(3).margin(1e-6));
	CHECK(sd.acceleration.frame == Frame::BODY);

	CHECK(sd.attitude.x() == Approx(0.1).margin(1e-6));
	CHECK(sd.attitude.y() == Approx(0.2).margin(1e-6));
	CHECK(sd.attitude.z() == Approx(0.3).margin(1e-6));

	CHECK(sd.angularRate.x() == Approx(0.2).margin(1e-6));
	CHECK(sd.angularRate.y() == Approx(0.3).margin(1e-6));
	CHECK(sd.angularRate.z() == Approx(0.4).margin(1e-6));
	CHECK(sd.angularRate.frame == Frame::BODY);

	CHECK(sd.groundSpeed == Approx(sqrt(29)).margin(1e-6));
	CHECK(sd.airSpeed == Approx(sqrt(29) + 0.3).margin(1e-6));
}

TEST_CASE("NED -> ENU -> NED Inertial")
{
	SensorData sd;
	sd.orientation = Orientation::NED;
	sd.position = {100, 200, 300};

	sd.velocity = Vector3({2, 3, 4});
	sd.velocity.frame = Frame::INERTIAL;

	sd.acceleration = Vector3({1, 2, 3});
	sd.acceleration.frame = Frame::INERTIAL;

	sd.attitude = Vector3({0.1, 0.2, 0.3});

	sd.angularRate = Vector3({0.2, 0.3, 0.4});
	sd.angularRate.frame = Frame::INERTIAL;
	sd.groundSpeed = sd.velocity.norm();
	sd.airSpeed = sd.groundSpeed + 0.3;

	// NOTE not calculaing AoA, but maybe we should
	ENU::convert(sd, Frame::INERTIAL, Frame::INERTIAL, Frame::INERTIAL);
	CHECK(sd.position.x() == Approx(200).margin(1e-6));
	CHECK(sd.position.y() == Approx(100).margin(1e-6));
	CHECK(sd.position.z() == Approx(-300).margin(1e-6));

	CHECK(sd.velocity.x() == Approx(3).margin(1e-6));
	CHECK(sd.velocity.y() == Approx(2).margin(1e-6));
	CHECK(sd.velocity.z() == Approx(-4).margin(1e-6));
	CHECK(sd.velocity.frame == Frame::INERTIAL);

	CHECK(sd.acceleration.x() == Approx(2).margin(1e-6));
	CHECK(sd.acceleration.y() == Approx(1).margin(1e-6));
	CHECK(sd.acceleration.z() == Approx(-3).margin(1e-6));
	CHECK(sd.acceleration.frame == Frame::INERTIAL);

	CHECK(sd.attitude.x() == Approx(0.1).margin(1e-6));
	CHECK(sd.attitude.y() == Approx(0.2).margin(1e-6));
	CHECK(sd.attitude.z() == Approx(-0.3 + degToRad(90)).margin(1e-6));

	CHECK(sd.angularRate.x() == Approx(0.2).margin(1e-6));
	CHECK(sd.angularRate.y() == Approx(0.3).margin(1e-6));
	CHECK(sd.angularRate.z() == Approx(-0.4).margin(1e-6));
	CHECK(sd.angularRate.frame == Frame::INERTIAL);

	CHECK(sd.groundSpeed == Approx(sqrt(29)).margin(1e-6));
	CHECK(sd.airSpeed == Approx(sqrt(29) + 0.3).margin(1e-6));

	NED::convert(sd, Frame::INERTIAL, Frame::INERTIAL, Frame::INERTIAL);
	CHECK(sd.position.x() == Approx(100).margin(1e-6));
	CHECK(sd.position.y() == Approx(200).margin(1e-6));
	CHECK(sd.position.z() == Approx(300).margin(1e-6));

	CHECK(sd.velocity.x() == Approx(2).margin(1e-6));
	CHECK(sd.velocity.y() == Approx(3).margin(1e-6));
	CHECK(sd.velocity.z() == Approx(4).margin(1e-6));
	CHECK(sd.velocity.frame == Frame::INERTIAL);

	CHECK(sd.acceleration.x() == Approx(1).margin(1e-6));
	CHECK(sd.acceleration.y() == Approx(2).margin(1e-6));
	CHECK(sd.acceleration.z() == Approx(3).margin(1e-6));
	CHECK(sd.acceleration.frame == Frame::INERTIAL);

	CHECK(sd.attitude.x() == Approx(0.1).margin(1e-6));
	CHECK(sd.attitude.y() == Approx(0.2).margin(1e-6));
	CHECK(sd.attitude.z() == Approx(0.3).margin(1e-6));

	CHECK(sd.angularRate.x() == Approx(0.2).margin(1e-6));
	CHECK(sd.angularRate.y() == Approx(0.3).margin(1e-6));
	CHECK(sd.angularRate.z() == Approx(0.4).margin(1e-6));
	CHECK(sd.angularRate.frame == Frame::INERTIAL);

	CHECK(sd.groundSpeed == Approx(sqrt(29)).margin(1e-6));
	CHECK(sd.airSpeed == Approx(sqrt(29) + 0.3).margin(1e-6));
}

TEST_CASE("NED -> ENU -> NED Body")
{
	SensorData sd;
	sd.orientation = Orientation::NED;
	sd.position = {100, 200, 300};

	sd.velocity = Vector3({2, 3, 4});
	sd.velocity.frame = Frame::BODY;

	sd.acceleration = Vector3({1, 2, 3});
	sd.acceleration.frame = Frame::BODY;

	sd.attitude = Vector3({0.1, 0.2, 0.3});

	sd.angularRate = Vector3({0.2, 0.3, 0.4});
	sd.angularRate.frame = Frame::BODY;
	sd.groundSpeed = sd.velocity.norm();
	sd.airSpeed = sd.groundSpeed + 0.3;

	// NOTE not calculaing AoA, but maybe we should
	ENU::convert(sd, Frame::BODY, Frame::BODY, Frame::BODY);
	CHECK(sd.position.x() == Approx(200).margin(1e-6));
	CHECK(sd.position.y() == Approx(100).margin(1e-6));
	CHECK(sd.position.z() == Approx(-300).margin(1e-6));

	CHECK(sd.velocity.x() == Approx(3).margin(1e-6));
	CHECK(sd.velocity.y() == Approx(2).margin(1e-6));
	CHECK(sd.velocity.z() == Approx(-4).margin(1e-6));
	CHECK(sd.velocity.frame == Frame::BODY);

	CHECK(sd.acceleration.x() == Approx(2).margin(1e-6));
	CHECK(sd.acceleration.y() == Approx(1).margin(1e-6));
	CHECK(sd.acceleration.z() == Approx(-3).margin(1e-6));
	CHECK(sd.acceleration.frame == Frame::BODY);

	CHECK(sd.attitude.x() == Approx(0.1).margin(1e-6));
	CHECK(sd.attitude.y() == Approx(0.2).margin(1e-6));
	CHECK(sd.attitude.z() == Approx(-0.3 + degToRad(90)).margin(1e-6));

	CHECK(sd.angularRate.x() == Approx(0.3).margin(1e-6));
	CHECK(sd.angularRate.y() == Approx(0.2).margin(1e-6));
	CHECK(sd.angularRate.z() == Approx(-0.4).margin(1e-6));
	CHECK(sd.angularRate.frame == Frame::BODY);

	CHECK(sd.groundSpeed == Approx(sqrt(29)).margin(1e-6));
	CHECK(sd.airSpeed == Approx(sqrt(29) + 0.3).margin(1e-6));

	NED::convert(sd, Frame::BODY, Frame::BODY, Frame::BODY);
	CHECK(sd.position.x() == Approx(100).margin(1e-6));
	CHECK(sd.position.y() == Approx(200).margin(1e-6));
	CHECK(sd.position.z() == Approx(300).margin(1e-6));

	CHECK(sd.velocity.x() == Approx(2).margin(1e-6));
	CHECK(sd.velocity.y() == Approx(3).margin(1e-6));
	CHECK(sd.velocity.z() == Approx(4).margin(1e-6));
	CHECK(sd.velocity.frame == Frame::BODY);

	CHECK(sd.acceleration.x() == Approx(1).margin(1e-6));
	CHECK(sd.acceleration.y() == Approx(2).margin(1e-6));
	CHECK(sd.acceleration.z() == Approx(3).margin(1e-6));
	CHECK(sd.acceleration.frame == Frame::BODY);

	CHECK(sd.attitude.x() == Approx(0.1).margin(1e-6));
	CHECK(sd.attitude.y() == Approx(0.2).margin(1e-6));
	CHECK(sd.attitude.z() == Approx(0.3).margin(1e-6));

	CHECK(sd.angularRate.x() == Approx(0.2).margin(1e-6));
	CHECK(sd.angularRate.y() == Approx(0.3).margin(1e-6));
	CHECK(sd.angularRate.z() == Approx(0.4).margin(1e-6));
	CHECK(sd.angularRate.frame == Frame::BODY);

	CHECK(sd.groundSpeed == Approx(sqrt(29)).margin(1e-6));
	CHECK(sd.airSpeed == Approx(sqrt(29) + 0.3).margin(1e-6));
}