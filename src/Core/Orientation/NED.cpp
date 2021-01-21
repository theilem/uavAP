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

			directionalToInertialENU(sd.velocity, sd.attitude);
			simpleFlipInertial(sd.velocity);
			directionalToFrameENU(sd.velocity, sd.attitude, Frame::BODY);

			directionalToInertialENU(sd.acceleration, sd.attitude);
			simpleFlipInertial(sd.acceleration);

			angularToInertialENU(sd.angularRate, sd.attitude);
			simpleFlipInertial(sd.angularRate);
			angularToBodyENU(sd.angularRate, sd.attitude);

			sd.orientation = Orientation::NED;
			break;
		case Orientation::NED:
			break;
		default:
			CPSLOG_ERROR << "Unknown sensor data orientation";
	}
}