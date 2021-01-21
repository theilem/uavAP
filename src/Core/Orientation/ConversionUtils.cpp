//
// Created by seedship on 1/21/21.
//

#include "uavAP/Core/Orientation/ConversionUtils.h"


void
directionalToInertialENU(FramedVector3& input, const Vector3& attitude)
{
	switch (input.frame)
	{
		case Frame::BODY:
			input = Eigen::AngleAxisd(-attitude[1], Vector3::UnitX()) * input;
		case Frame::VEHICLE_2:
			input = Eigen::AngleAxisd(-attitude[0], Vector3::UnitY()) * input;
		case Frame::VEHICLE_1:
			input = Eigen::AngleAxisd(-attitude[2], Vector3::UnitZ()) * input;
			input.frame = Frame::INERTIAL;
			break;
		case Frame::INERTIAL:
			break;
		default:
			CPSLOG_ERROR << "Unknown frame!";
	}
}

void
directionalToInertialNED(FramedVector3& input, const Vector3& attitude)
{
	switch (input.frame)
	{
		case Frame::BODY:
			input = Eigen::AngleAxisd(-attitude[0], Vector3::UnitX()) * input;
		case Frame::VEHICLE_2:
			input = Eigen::AngleAxisd(-attitude[1], Vector3::UnitY()) * input;
		case Frame::VEHICLE_1:
			input = Eigen::AngleAxisd(-attitude[2], Vector3::UnitZ()) * input;
			input.frame = Frame::INERTIAL;
			break;
		case Frame::INERTIAL:
			break;
		default:
			CPSLOG_ERROR << "Unknown frame!";
	}
}

void
directionalToFrameENU(FramedVector3& input, const Vector3& attitude, Frame target)
{
	if (input.frame == target)
	{
		return;
	}
	if (input.frame != Frame::INERTIAL)
	{
		directionalToInertialENU(input, attitude);
	}

	switch (target)
	{
		case Frame::INERTIAL:
			break;
		case Frame::VEHICLE_1:
			input = Eigen::AngleAxisd(attitude[2], Vector3::UnitZ()) * input;
			input.frame = Frame::VEHICLE_1;
			break;
		case Frame::VEHICLE_2:
			input = Eigen::AngleAxisd(attitude[2], Vector3::UnitZ()) *
					Eigen::AngleAxisd(attitude[0], Vector3::UnitY()) * input;
			input.frame = Frame::VEHICLE_2;
			break;
		case Frame::BODY:
			input = Eigen::AngleAxisd(attitude[2], Vector3::UnitZ()) *
					Eigen::AngleAxisd(attitude[0], Vector3::UnitY()) *
					Eigen::AngleAxisd(attitude[1], Vector3::UnitX()) * input;
			input.frame = Frame::BODY;
			break;
		default:
			CPSLOG_ERROR << "Unknown frame!";
	}
}

void
directionalToFrameNED(FramedVector3& input, const Vector3& attitude, Frame target)
{
	if (input.frame == target)
	{
		return;
	}
	if (input.frame != Frame::INERTIAL)
	{
		directionalToInertialNED(input, attitude);
	}

	switch (target)
	{
		case Frame::INERTIAL:
			break;
		case Frame::VEHICLE_1:
			input = Eigen::AngleAxisd(attitude[2], Vector3::UnitZ()) * input;
			input.frame = Frame::VEHICLE_1;
			break;
		case Frame::VEHICLE_2:
			input = Eigen::AngleAxisd(attitude[2], Vector3::UnitZ()) *
					Eigen::AngleAxisd(attitude[1], Vector3::UnitY()) * input;
			input.frame = Frame::VEHICLE_2;
			break;
		case Frame::BODY:
			input = Eigen::AngleAxisd(attitude[2], Vector3::UnitZ()) *
					Eigen::AngleAxisd(attitude[1], Vector3::UnitY()) *
					Eigen::AngleAxisd(attitude[0], Vector3::UnitX()) * input;
			input.frame = Frame::BODY;
			break;
		default:
			CPSLOG_ERROR << "Unknown frame!";
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