//
// Created by seedship on 1/21/21.
//

#include <cpsCore/Utilities/Test/TestInfo.h>
#include <uavAP/Core/SensorData.h>
#include <uavAP/Core/Orientation/ENU.h>
#include <uavAP/Core/Orientation/NED.h>



TEST_CASE("NED conversion test 1")
{
	SensorData sd;
	sd.orientation = Orientation::ENU;
	
	sd.position = {1, 2, 3};
//	sd.attitude = degToRad({45, 45, 45});
//	sd.velocity = Vector3({0, 1, 0});
	sd.velocity.frame = Frame::BODY;

	sd.angleOfAttack = 1;
	
	NED::convert(sd, Frame::INERTIAL);
	assert(sd.position == Vector3({2, 1, -3}));

	//TODO check velocity, acceleration and angular rates and make sure they are correct
//	assert(sd.velocity[0] == Approx(0.5).margin(1e-6));
//	assert(sd.velocity[1] == Approx(0.5).margin(1e-6));
//	assert(sd.velocity[2] == Approx(-1./sqrt(2)).margin(1e-6));

	assert(sd.angleOfAttack == -1);
	assert(sd.orientation == Orientation::NED);
}

TEST_CASE("NED conversion test 2")
{
	SensorData sd;
	sd.orientation = Orientation::ENU;

	sd.position = {1, 2, 3};
//	sd.attitude = degToRad({45, 45, 45});
//	sd.velocity = Vector3({0, 1, 0});
	sd.velocity.frame = Frame::BODY;

	sd.angleOfAttack = 1;

	NED::convert(sd, Frame::BODY);
	assert(sd.position == Vector3({2, 1, -3}));

//	assert(sd.velocity[0] == Approx(1).margin(1e-6));
//	assert(sd.velocity[1] == Approx(0).margin(1e-6));
//	assert(sd.velocity[2] == Approx(0).margin(1e-6));

	assert(sd.angleOfAttack == -1);
	assert(sd.orientation == Orientation::NED);
}

TEST_CASE("ENU conversion test")
{
	SensorData sd;
	sd.orientation = Orientation::NED;
	ENU::convert(sd);
	assert(sd.orientation == Orientation::ENU);
}
