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
			directionalConversion(sd.velocity, sd.attitude, Frame::BODY, Orientation::NED);

			directionalConversion(sd.acceleration, sd.attitude, Frame::INERTIAL, Orientation::ENU);
			simpleFlipInertial(sd.acceleration);

			angularConversion(sd.angularRate, sd.attitude, Frame::INERTIAL, Orientation::NED);
			simpleFlipInertial(sd.angularRate);
			angularConversion(sd.angularRate, sd.attitude, Frame::BODY, Orientation::ENU);

			sd.orientation = Orientation::ENU;
			break;
		case Orientation::NED:
			break;
		default:
			CPSLOG_ERROR << "Unknown sensor data orientation";
	}
}