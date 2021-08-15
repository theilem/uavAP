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

	sd.uvw_dot = {
		cos(psi)*cos(theta)*Ax - cos(theta)*vz*theta_dot - sin(theta)*Az + cos(theta)*sin(psi)*Ay + cos(psi)*cos(theta)*vy*psi_dot - cos(theta)*sin(psi)*vx*psi_dot - cos(psi)*sin(theta)*vx*theta_dot - sin(psi)*sin(theta)*vy*theta_dot,
		vx*(sin(phi)*sin(psi)*phi_dot - cos(phi)*cos(psi)*psi_dot + cos(phi)*cos(psi)*sin(theta)*phi_dot + cos(psi)*cos(theta)*sin(phi)*theta_dot - sin(phi)*sin(psi)*sin(theta)*psi_dot) + vy*(cos(phi)*sin(psi)*sin(theta)*phi_dot - cos(phi)*sin(psi)*psi_dot - cos(psi)*sin(phi)*phi_dot + cos(psi)*sin(phi)*sin(theta)*psi_dot + cos(theta)*sin(phi)*sin(psi)*theta_dot) - (cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*Ax + (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*Ay + cos(theta)*sin(phi)*Az + cos(phi)*cos(theta)*vz*phi_dot - sin(phi)*sin(theta)*vz*theta_dot,
		vx*(cos(phi)*sin(psi)*phi_dot + cos(psi)*sin(phi)*psi_dot - cos(psi)*sin(phi)*sin(theta)*phi_dot - cos(phi)*sin(psi)*sin(theta)*psi_dot + cos(phi)*cos(psi)*cos(theta)*theta_dot) + vy*(sin(phi)*sin(psi)*psi_dot - cos(phi)*cos(psi)*phi_dot + cos(phi)*cos(psi)*sin(theta)*psi_dot + cos(phi)*cos(theta)*sin(psi)*theta_dot - sin(phi)*sin(psi)*sin(theta)*phi_dot) + (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*Ax - (cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*Ay + cos(phi)*cos(theta)*Az - cos(theta)*sin(phi)*vz*phi_dot - cos(phi)*sin(theta)*vz*theta_dot
	};
}