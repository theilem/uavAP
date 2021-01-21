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

			directionalToInertialNED(sd.velocity, sd.attitude);
			simpleFlipInertial(sd.velocity);
			directionalToFrameNED(sd.velocity, sd.attitude, Frame::BODY);

			directionalToInertialNED(sd.acceleration, sd.attitude);
			simpleFlipInertial(sd.acceleration);

			angularToInertialNED(sd.angularRate, sd.attitude);
			simpleFlipInertial(sd.angularRate);
			angularToBodyNED(sd.angularRate, sd.attitude);

			sd.orientation = Orientation::ENU;
			break;
		default:
			CPSLOG_ERROR << "Unknown sensor data orientation";
	}
}
