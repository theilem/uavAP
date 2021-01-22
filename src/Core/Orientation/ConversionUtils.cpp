//
// Created by seedship on 1/21/21.
//

#include "uavAP/Core/Orientation/ConversionUtils.h"

// Helper Functions
static void
upConversionENU(FramedVector3& input, const Vector3& attitude, Frame target);

static void
downConversionENU(FramedVector3& input, const Vector3& attitude, Frame target);

static void
upConversionNED(FramedVector3& input, const Vector3& attitude, Frame target);

static void
downConversionNED(FramedVector3& input, const Vector3& attitude, Frame target);

/**
 * Converts angular rate into inertial frame for ENU
 * (dot Phi, dot Theta, dot Psi)
 * @param angularRate
 * @param attitude
 */
static void
angularToInertialENU(FramedVector3& angularRate, const Vector3& attitude);

/**
 * Converts inertial angular rate (dot phi, dot theta, dot psi) to body angular rate (pqr) for ENU
 * @param angularRate
 * @param attitude
 */
static void
angularToBodyENU(FramedVector3& angularRate, const Vector3& attitude);

/**
 * Converts angular rate into inertial frame for NED
 * (dot Phi, dot Theta, dot Psi)
 * @param angularRate
 * @param attitude
 */
static void
angularToInertialNED(FramedVector3& angularRate, const Vector3& attitude);

/**
 * Converts inertial angular rate (dot phi, dot theta, dot psi) to body angular rate (pqr) for NED
 * @param angularRate
 * @param attitude
 */
static void
angularToBodyNED(FramedVector3& angularRate, const Vector3& attitude);

void
directionalConversion(FramedVector3& input, const Vector3& attitude, Frame target, Orientation orientation)
{
	if (input.frame == target)
	{
		return;
	}
	else if (input.frame < target)
	{
		if (orientation == Orientation::NED)
			upConversionNED(input, attitude, target);
		else
			upConversionENU(input, attitude, target);
	}
	else
	{
		if (orientation == Orientation::NED)
			downConversionNED(input, attitude, target);
		else
			downConversionENU(input, attitude, target);
	}
}


void
angularConversion(FramedVector3& angularRate, const Vector3& attitude, Frame target, Orientation orientation)
{
	if (angularRate.frame == target)
	{
		return;
	}
	if (target == Frame::VEHICLE_1 || target == Frame::VEHICLE_2 || angularRate.frame == Frame::VEHICLE_1 ||
		angularRate.frame == Frame::VEHICLE_2)
	{
		CPSLOG_ERROR << "Vehicle 1/Vehicle 2 frame is undefined for angular rate.";
		return;
	}
	if (orientation == Orientation::NED)
	{
		if (target == Frame::BODY)
		{
			angularToBodyNED(angularRate, attitude);
		}
		else
		{
			angularToInertialNED(angularRate, attitude);
		}
	}
	else
	{
		if (target == Frame::BODY)
		{
			angularToBodyENU(angularRate, attitude);
		}
		else
		{
			angularToInertialENU(angularRate, attitude);
		}
	}
}

static void
upConversionENU(FramedVector3& input, const Vector3& attitude, Frame target)
{
	switch (input.frame)
	{
		case Frame::INERTIAL:
			input = Eigen::AngleAxisd(-attitude[2], Vector3::UnitZ()) * input;
			input.frame = Frame::VEHICLE_1;
		case Frame::VEHICLE_1:
			if (target == Frame::VEHICLE_1)
				break;
			input = Eigen::AngleAxisd(-attitude[1], Vector3::UnitX()) * input;
			input.frame = Frame::VEHICLE_2;
		case Frame::VEHICLE_2:
			if (target == Frame::VEHICLE_2)
				break;
			input = Eigen::AngleAxisd(attitude[0], Vector3::UnitY()) * input;
			input.frame = Frame::BODY;
		case Frame::BODY:
			if (target == Frame::BODY)
				break;
		default:
			CPSLOG_ERROR << "Unknown Frame";
	}
}

static void
downConversionENU(FramedVector3& input, const Vector3& attitude, Frame target)
{
	switch (input.frame)
	{
		case Frame::BODY:
			input = Eigen::AngleAxisd(-attitude[0], Vector3::UnitY()) * input;
			input.frame = Frame::VEHICLE_2;
		case Frame::VEHICLE_2:
			if (target == Frame::VEHICLE_2)
				break;
			input = Eigen::AngleAxisd(attitude[1], Vector3::UnitX()) * input;
			input.frame = Frame::VEHICLE_1;
		case Frame::VEHICLE_1:
			if (target == Frame::VEHICLE_1)
				break;
			input = Eigen::AngleAxisd(attitude[2], Vector3::UnitZ()) * input;
			input.frame = Frame::INERTIAL;
		case Frame::INERTIAL:
			if (target == Frame::INERTIAL)
				break;
		default:
			CPSLOG_ERROR << "Unknown Frame";
	}
}

static void
upConversionNED(FramedVector3& input, const Vector3& attitude, Frame target)
{
	switch (input.frame)
	{
		case Frame::INERTIAL:
			input = Eigen::AngleAxisd(-attitude[2], Vector3::UnitZ()) * input;
			input.frame = Frame::VEHICLE_1;
		case Frame::VEHICLE_1:
			if (target == Frame::VEHICLE_1)
				break;
			input = Eigen::AngleAxisd(-attitude[1], Vector3::UnitY()) * input;
			input.frame = Frame::VEHICLE_2;
		case Frame::VEHICLE_2:
			if (target == Frame::VEHICLE_2)
				break;
			input = Eigen::AngleAxisd(-attitude[0], Vector3::UnitX()) * input;
			input.frame = Frame::BODY;
		case Frame::BODY:
			if (target == Frame::BODY)
				break;
		default:
			CPSLOG_ERROR << "Unknown Frame";
	}
}

static void
downConversionNED(FramedVector3& input, const Vector3& attitude, Frame target)
{
	switch (input.frame)
	{
		case Frame::BODY:
			input = Eigen::AngleAxisd(attitude[0], Vector3::UnitX()) * input;
			input.frame = Frame::VEHICLE_2;
		case Frame::VEHICLE_2:
			if (target == Frame::VEHICLE_2)
				break;
			input = Eigen::AngleAxisd(attitude[1], Vector3::UnitY()) * input;
			input.frame = Frame::VEHICLE_1;
		case Frame::VEHICLE_1:
			if (target == Frame::VEHICLE_1)
				break;
			input = Eigen::AngleAxisd(attitude[2], Vector3::UnitZ()) * input;
			input.frame = Frame::INERTIAL;
		case Frame::INERTIAL:
			if (target == Frame::INERTIAL)
				break;
		default:
			CPSLOG_ERROR << "Unknown Frame";
	}
}

void
angularToInertialENU(FramedVector3& angularRate, const Vector3& attitude)
{
	const auto& p = angularRate[0];
	const auto& q = angularRate[1];
	const auto& r = angularRate[2];
	const auto& phi = attitude[0];
	const auto& theta = attitude[1];
	switch (angularRate.frame)
	{
		case Frame::BODY:
			angularRate = Vector3({
										  q - r * tan(theta) * cos(phi) - p * tan(theta) * sin(phi),
										  p * cos(phi) - r * sin(phi),
										  (r * cos(phi) + p * sin(phi)) / cos(theta)
								  });
			angularRate.frame = Frame::INERTIAL;
			break;
		case Frame::INERTIAL:
			break;
		case Frame::VEHICLE_1:
			CPSLOG_ERROR << "Vehicle 1 frame is undefined for angular rates";
			break;
		case Frame::VEHICLE_2:
			CPSLOG_ERROR << "Vehicle 2 frame is undefined for angular rates";
			break;
		default:
			CPSLOG_ERROR << "Unknown frame!";
	}
}

void
angularToBodyENU(FramedVector3& angularRate, const Vector3& attitude)
{
	const auto& dphi = angularRate[0];
	const auto& dtheta = angularRate[1];
	const auto& dpsi = angularRate[2];
	const auto& phi = attitude[0];
	const auto& theta = attitude[1];
	switch (angularRate.frame)
	{
		case Frame::INERTIAL:
			angularRate = Vector3({
										  dpsi * sin(phi) * cos(theta) - dtheta * cos(phi),
										  dpsi * sin(theta) - dphi,
										  dpsi * cos(phi) * cos(theta) - dtheta * sin(phi)
								  });
			angularRate.frame = Frame::BODY;
			break;
		case Frame::BODY:
			break;
		case Frame::VEHICLE_1:
			CPSLOG_ERROR << "Vehicle 1 frame is undefined for angular rates";
			break;
		case Frame::VEHICLE_2:
			CPSLOG_ERROR << "Vehicle 2 frame is undefined for angular rates";
			break;
		default:
			CPSLOG_ERROR << "Unknown frame!";
	}
}

void
angularToInertialNED(FramedVector3& angularRate, const Vector3& attitude)
{
	const auto& p = angularRate[0];
	const auto& q = angularRate[1];
	const auto& r = angularRate[2];
	const auto& phi = attitude[0];
	const auto& theta = attitude[1];
	switch (angularRate.frame)
	{
		case Frame::BODY:
			angularRate = Vector3({
										  p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta),
										  q * cos(phi) - r * sin(phi),
										  (q * cos(phi) + r * cos(phi)) / cos(theta)
								  });
			angularRate.frame = Frame::INERTIAL;
			break;
		case Frame::INERTIAL:
			break;
		case Frame::VEHICLE_1:
			CPSLOG_ERROR << "Vehicle 1 frame is undefined for angular rates";
			break;
		case Frame::VEHICLE_2:
			CPSLOG_ERROR << "Vehicle 2 frame is undefined for angular rates";
			break;
		default:
			CPSLOG_ERROR << "Unknown frame!";
	}
}

void
angularToBodyNED(FramedVector3& angularRate, const Vector3& attitude)
{
	const auto& dphi = angularRate[0];
	const auto& dtheta = angularRate[1];
	const auto& dpsi = angularRate[2];
	const auto& phi = attitude[0];
	const auto& theta = attitude[1];
	switch (angularRate.frame)
	{
		case Frame::INERTIAL:
			angularRate = Vector3({
										  dphi - dpsi * sin(theta),
										  dtheta * cos(phi) + dpsi * cos(theta) * sin(phi),
										  dpsi * cos(theta) * cos(phi) - dtheta * sin(phi)
								  });
			angularRate.frame = Frame::BODY;
			break;
		case Frame::BODY:
			break;
		case Frame::VEHICLE_1:
			CPSLOG_ERROR << "Vehicle 1 frame is undefined for angular rates";
			break;
		case Frame::VEHICLE_2:
			CPSLOG_ERROR << "Vehicle 2 frame is undefined for angular rates";
			break;
		default:
			CPSLOG_ERROR << "Unknown frame!";
	}
}

void
simpleFlipInertial(Vector3& input)
{
	std::swap(input.x(), input.y());
	input.z() *= -1;
}