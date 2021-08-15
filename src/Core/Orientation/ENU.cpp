//
// Created by seedship on 1/21/21.
//

#include "uavAP/Core/Orientation/ENU.h"
#include "uavAP/Core/Orientation/NED.h"
#include "uavAP/Core/Orientation/ConversionUtils.h"

void
ENU::convert(SensorData& sd, Frame velocityFrame, Frame accelerationFrame, Frame angularRateFrame)
{
	switch(sd.orientation){
		case Orientation::ENU:
			break;
		case Orientation::NED:
			// Position is unframed, simple flip
			simpleFlipInertial(sd.position);
			// I think this can also be simply flipped
			simpleFlipInertial(sd.uvw_dot);

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

			directionalConversion(sd.velocity, sd.attitude, velocityFrame, Orientation::ENU);
			directionalConversion(sd.acceleration, sd.attitude, accelerationFrame, Orientation::ENU);
			angularConversion(sd.angularRate, sd.attitude, angularRateFrame, Orientation::ENU);

			// AoA is negated
			sd.angleOfAttack = -sd.angleOfAttack;

			sd.orientation = Orientation::ENU;
			break;
		default:
			CPSLOG_ERROR << "Unknown sensor data orientation";
	}
}

void
ENU::setUVWDot(SensorData& sd)
{
	SensorData sd_ned = sd;
	NED::convert(sd_ned, Frame::INERTIAL, Frame::INERTIAL, Frame::INERTIAL);
	NED::setUVWDot(sd_ned);
	sd.uvw_dot = sd_ned.uvw_dot;
	simpleFlipInertial(sd.uvw_dot);
}

