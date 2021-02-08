//
// Created by seedship on 1/21/21.
//

#include "uavAP/Core/Orientation/NED.h"
#include "uavAP/Core/Orientation/ConversionUtils.h"

void
NED::convert(SensorData& sd, Frame velocityFrame, Frame accelerationFrame, Frame angularRateFrame)
{
	switch(sd.orientation){
		case Orientation::ENU:

			// Position is unframed, simple flip
			simpleFlipInertial(sd.position);


			// Change velocity to inertial and flip
			directionalConversion(sd.velocity, sd.attitude, Frame::INERTIAL, Orientation::ENU);
			simpleFlipInertial(sd.velocity);

			// Change acceleration to inertial and flip
			directionalConversion(sd.acceleration, sd.attitude, Frame::INERTIAL, Orientation::ENU);
			simpleFlipInertial(sd.acceleration);

			// Yaw needs to be negated
			sd.attitude[2] = boundAngleRad(-sd.attitude[2] + degToRad(90));
			sd.angularRate[2] *= -1;

			// If we are in body frame, we need to change angular rate from PQR -> QPR (R is already negated)
			if(sd.angularRate.frame == Frame::BODY)
			{
				std::swap(sd.angularRate[0], sd.angularRate[1]);
			}

			directionalConversion(sd.velocity, sd.attitude, velocityFrame, Orientation::NED);
			directionalConversion(sd.acceleration, sd.attitude, accelerationFrame, Orientation::NED);
			angularConversion(sd.angularRate, sd.attitude, angularRateFrame, Orientation::NED);

			// AoA is negated
			sd.angleOfAttack = -sd.angleOfAttack;

			sd.orientation = Orientation::NED;
			break;
		case Orientation::NED:
			break;
		default:
			CPSLOG_ERROR << "Unknown sensor data orientation";
	}
}