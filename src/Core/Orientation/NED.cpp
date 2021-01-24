//
// Created by seedship on 1/21/21.
//

#include "uavAP/Core/Orientation/NED.h"
#include "uavAP/Core/Orientation/ConversionUtils.h"

void
NED::convert(SensorData& sd)
{
	switch(sd.orientation){
		case Orientation::ENU:
			simpleFlipInertial(sd.position);

			directionalConversion(sd.velocity, sd.attitude, Frame::INERTIAL, Orientation::ENU);
			simpleFlipInertial(sd.velocity);
			directionalConversion(sd.acceleration, sd.attitude, Frame::INERTIAL, Orientation::ENU);
			simpleFlipInertial(sd.acceleration);
			angularConversion(sd.angularRate, sd.attitude, Frame::INERTIAL, Orientation::ENU);

			sd.attitude[2] = boundAngleRad(-sd.attitude[2] + degToRad(90));
			sd.angularRate[2] *= -1;

			directionalConversion(sd.velocity, sd.attitude, Frame::BODY, Orientation::NED);
			angularConversion(sd.angularRate, sd.attitude, Frame::BODY, Orientation::NED);

			sd.angleOfAttack = -sd.angleOfAttack;

			sd.orientation = Orientation::NED;
			break;
		case Orientation::NED:
			break;
		default:
			CPSLOG_ERROR << "Unknown sensor data orientation";
	}
}