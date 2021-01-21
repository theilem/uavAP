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
			simpleFlipIntertial(sd.position);

			directionalToInertialNED(sd.velocity, sd.attitude);
			simpleFlipIntertial(sd.velocity.data);

			directionalToInertialNED(sd.acceleration, sd.attitude);
			simpleFlipIntertial(sd.acceleration.data);

			angularToInertialNED(sd.angularRate, sd.attitude);
			simpleFlipIntertial(sd.angularRate.data);

			sd.orientation = Orientation::ENU;
			break;
		default:
			CPSLOG_ERROR << "Unknown sensor data orientation";
	}
}
