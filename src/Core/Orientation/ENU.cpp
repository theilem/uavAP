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
			// Position is unframed, simple flip
			simpleFlipInertial(sd.position);

			// Change velocity to inertial and flip
			directionalConversion(sd.velocity, sd.attitude, Frame::INERTIAL, Orientation::NED);
			simpleFlipInertial(sd.velocity);

			// Change acceleration to inertial and flip
			directionalConversion(sd.acceleration, sd.attitude, Frame::INERTIAL, Orientation::NED);
			simpleFlipInertial(sd.acceleration);

			// Yaw needs to be negated
			sd.attitude[2] = boundAngleRad(-sd.attitude[2] + degToRad(90));
			sd.angularRate[2] *= -1;

			// If we are in body frame, we need to change angular rate from PQR -> QPR (R is already negated)
			if(sd.angularRate.frame == Frame::BODY)
			{
				std::swap(sd.angularRate[0], sd.angularRate[1]);
			}

			directionalConversion(sd.velocity, sd.attitude, Frame::BODY, Orientation::ENU);
			angularConversion(sd.angularRate, sd.attitude, Frame::BODY, Orientation::ENU);

			// AoA is negated
			sd.angleOfAttack = -sd.angleOfAttack;

			sd.orientation = Orientation::ENU;
			break;
		default:
			CPSLOG_ERROR << "Unknown sensor data orientation";
	}
}
