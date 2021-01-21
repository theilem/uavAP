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
			input.data = Eigen::AngleAxisd(-attitude[1], Vector3::UnitX()) * input.data;
		case Frame::VEHICLE_2:
			input.data = Eigen::AngleAxisd(-attitude[0], Vector3::UnitY()) * input.data;
		case Frame::VEHICLE_1:
			input.data = Eigen::AngleAxisd(-attitude[2], Vector3::UnitZ()) * input.data;
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
			input.data = Eigen::AngleAxisd(-attitude[0], Vector3::UnitX()) * input.data;
		case Frame::VEHICLE_2:
			input.data = Eigen::AngleAxisd(-attitude[1], Vector3::UnitY()) * input.data;
		case Frame::VEHICLE_1:
			input.data = Eigen::AngleAxisd(-attitude[2], Vector3::UnitZ()) * input.data;
			input.frame = Frame::INERTIAL;
			break;
		case Frame::INERTIAL:
			break;
		default:
			CPSLOG_ERROR << "Unknown frame!";
	}
}

void
angularToInertialENU(FramedVector3& angularRate, const Vector3& attitude)
{
	const auto &p = angularRate.data[0];
	const auto &q = angularRate.data[1];
	const auto &r = angularRate.data[2];
	const auto &phi = attitude[0];
	const auto &theta = attitude[1];
	switch (angularRate.frame)
	{
		case Frame::BODY:
			angularRate.data = {
					q - r*tan(theta)*cos(phi) - p*tan(theta)*sin(phi),
					p * cos(phi) - r * sin(phi),
					(r*cos(phi)+p*sin(phi))/cos(theta)
			};
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
	const auto &dphi = angularRate.data[0];
	const auto &dtheta = angularRate.data[1];
	const auto &dpsi = angularRate.data[2];
	const auto &phi = attitude[0];
	const auto &theta = attitude[1];
	switch (angularRate.frame)
	{
		case Frame::INERTIAL:
			angularRate.data = {
					dpsi*sin(phi)*cos(theta) - dtheta*cos(phi),
					dpsi * sin(theta) - dphi,
					dpsi*cos(phi)*cos(theta)-dtheta*sin(phi)
			};
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
	const auto &p = angularRate.data[0];
	const auto &q = angularRate.data[1];
	const auto &r = angularRate.data[2];
	const auto &phi = attitude[0];
	const auto &theta = attitude[1];
	switch (angularRate.frame)
	{
		case Frame::BODY:
			angularRate.data = {
					p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta),
					q * cos(phi) - r * sin(phi),
					(q*cos(phi)+r*cos(phi))/cos(theta)
			};
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
	const auto &dphi = angularRate.data[0];
	const auto &dtheta = angularRate.data[1];
	const auto &dpsi = angularRate.data[2];
	const auto &phi = attitude[0];
	const auto &theta = attitude[1];
	switch (angularRate.frame)
	{
		case Frame::INERTIAL:
			angularRate.data = {
					dphi - dpsi*sin(theta),
					dtheta * cos(phi) + dpsi * cos(theta) * sin(phi),
					dpsi * cos(theta) * cos(phi) - dtheta * sin(phi)
			};
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