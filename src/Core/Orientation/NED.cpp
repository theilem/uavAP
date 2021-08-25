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
			// I think this can also be simply flipped
			simpleFlipInertial(sd.uvw_dot);


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

void
NED::setUVWDot(SensorData& sd)
{
	FramedVector3 inertialAccl = sd.acceleration;
	directionalConversion(inertialAccl, sd.attitude, Frame::INERTIAL, Orientation::NED);

	FramedVector3 omega = sd.angularRate;
	angularConversion(omega, sd.attitude, Frame::INERTIAL, Orientation::NED);

	FramedVector3 velocity = sd.velocity;
	directionalConversion(velocity, sd.attitude, Frame::INERTIAL, Orientation::NED);

	auto phi = sd.attitude.x();
	auto theta = sd.attitude.y();
	auto psi = sd.attitude.z();

	auto Ax = inertialAccl.x();
	auto Ay = inertialAccl.y();
	auto Az = inertialAccl.z();

	auto phi_dot = omega.x();
	auto theta_dot = omega.y();
	auto psi_dot = omega.z();

	auto vx = velocity.x();
	auto vy = velocity.y();
	auto vz = velocity.z();
	
	auto cos_phi = cos(phi);
	auto cos_theta = cos(theta);
	auto cos_psi = cos(psi);

	auto sin_phi = sin(phi);
	auto sin_theta = sin(theta);
	auto sin_psi = sin(psi);

	sd.uvw_dot = {
			cos_psi * cos_theta * Ax - cos_theta * vz * theta_dot - sin_theta * Az + cos_theta * sin_psi * Ay + cos_psi * cos_theta * vy * psi_dot - cos_theta * sin_psi * vx * psi_dot - cos_psi * sin_theta * vx * theta_dot - sin_psi * sin_theta * vy * theta_dot,
		vx*(sin_phi*sin_psi*phi_dot - cos_phi * cos_psi * psi_dot + cos_phi * cos_psi * sin_theta * phi_dot + cos_psi * cos_theta * sin_phi * theta_dot - sin_phi * sin_psi * sin_theta * psi_dot) + vy * (cos_phi * sin_psi * sin_theta * phi_dot - cos_phi * sin_psi * psi_dot - cos_psi * sin_phi * phi_dot + cos_psi * sin_phi * sin_theta * psi_dot + cos_theta * sin_phi * sin_psi * theta_dot) - (cos_phi * sin_psi - cos_psi * sin_phi * sin_theta) * Ax + (cos_phi * cos_psi + sin_phi * sin_psi * sin_theta) * Ay + cos_theta * sin_phi * Az + cos_phi * cos_theta * vz * phi_dot - sin_phi * sin_theta * vz * theta_dot,
		vx*(cos_phi*sin_psi*phi_dot + cos_psi * sin_phi * psi_dot - cos_psi * sin_phi * sin_theta * phi_dot - cos_phi * sin_psi * sin_theta * psi_dot + cos_phi * cos_psi * cos_theta * theta_dot) + vy * (sin_phi * sin_psi * psi_dot - cos_phi * cos_psi * phi_dot + cos_phi * cos_psi * sin_theta * psi_dot + cos_phi * cos_theta * sin_psi * theta_dot - sin_phi * sin_psi * sin_theta * phi_dot) + (sin_phi * sin_psi + cos_phi * cos_psi * sin_theta) * Ax - (cos_psi * sin_phi - cos_phi * sin_psi * sin_theta) * Ay + cos_phi * cos_theta * Az - cos_theta * sin_phi * vz * phi_dot - cos_phi * sin_theta * vz * theta_dot
	};
}