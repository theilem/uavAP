//
// Created by seedship on 1/21/21.
//

#include "uavAP/Core/Orientation/ENU.h"
#include "uavAP/Core/Orientation/ConversionUtils.h"

void
ENU::convert(SensorData& sd)
{
	switch(sd.orientation){
		case Orientation::ENU:
			break;
		case Orientation::NED:
			simpleFlipInertial(sd.position);

			directionalConversion(sd.velocity, sd.attitude, Frame::INERTIAL, Orientation::NED);
			simpleFlipInertial(sd.velocity);
			directionalConversion(sd.acceleration, sd.attitude, Frame::INERTIAL, Orientation::NED);
			simpleFlipInertial(sd.acceleration);
			angularConversion(sd.angularRate, sd.attitude, Frame::INERTIAL, Orientation::NED);

			sd.attitude[2] = boundAngleRad(-sd.attitude[2] + degToRad(90));
			sd.angularRate[2] *= -1;

			directionalConversion(sd.velocity, sd.attitude, Frame::BODY, Orientation::ENU);
			angularConversion(sd.angularRate, sd.attitude, Frame::BODY, Orientation::ENU);

			sd.angleOfAttack = -sd.angleOfAttack;

			sd.orientation = Orientation::ENU;
			break;
		default:
			CPSLOG_ERROR << "Unknown sensor data orientation";
	}
}
