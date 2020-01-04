////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/*
 * ConstRollRateModel.cpp
 *
 *  Created on: Aug 15, 2018
 *      Author: mircot
 */
#include <acb_hypgeom.h>
#include <uavAP/Core/Frames/InertialFrame.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>
#include <uavAP/Core/SensorData.h>
#include <uavAP/MissionControl/Geofencing/ConstRollRateModel.h>
#include <uavAP/MissionControl/Polygon.h>
#include <cmath>
#include <complex>

ConstRollRateModel::ConstRollRateModel() :
		currentRoll_(0), factor_(0), velocity_(0), yawEndLeft_(0), yawEndRight_(0), yawrateOrbit_(0)
{
	acb_init(aLeft_);
	acb_init(aRight_);
	acb_init(b_);
	acb_init(query_);
	acb_init(queryRes_);

	acb_set_d(b_, 0.5);


	Control::LowPassFilterParams p;
	p.cutOffFrequency = 20;
	airspeedFilter_.setParams(p);
}

ConstRollRateModel::~ConstRollRateModel()
{
	acb_clear(aLeft_);
	acb_clear(aRight_);
	acb_clear(b_);
	acb_clear(query_);
	acb_clear(queryRes_);
}

bool
ConstRollRateModel::updateModel(const SensorData& data, const WindInfo& wind)
{
	std::unique_lock<std::mutex> lock(queryMutex_, std::try_to_lock);
	if (!lock.owns_lock())
	{
		APLOG_DEBUG << "Data in use. Will update next time.";
		return false;
	}
	airspeedFilter_.update(data.airSpeed, 0.05); //Todo hardcoded frequency
	velocity_ = airspeedFilter_.getValue();
	currentRoll_ = data.attitude.x();
	acb_set_d_d(aLeft_, 0.5, -params.g() / (2 * velocity_ * params.rollRate()));
	acb_set_d_d(aRight_, 0.5, params.g() / (2 * velocity_ * params.rollRate()));
	acb_one(query_);
	acb_hypgeom_beta_lower(queryRes_, aLeft_, b_, query_, 0, params.precision());
	betaCompleteLeft_ = toVector(queryRes_);
	acb_hypgeom_beta_lower(queryRes_, aRight_, b_, query_, 0, params.precision());
	betaCompleteRight_ = toVector(queryRes_);

	factor_ = velocity_ / (2 * params.rollRate());

	//Calculate Centers
	//Left -> rollRate negative, rollMax negative
	const FloatingType& roll = currentRoll_;
	const FloatingType& heading = data.attitude[2];
	const Vector3& pos = data.position;

	//Reset frames

	frameLeft_.setOrigin(Vector3(0, 0, 0));
	frameLeft_.setYaw(0);
	frameRight_.setOrigin(Vector3(0, 0, 0));
	frameRight_.setYaw(0);

	Vector3 c0Left = calculatePoint(roll, RollDirection::LEFT);
	FloatingType psi0Left = calculateYaw(roll, RollDirection::LEFT);
	Vector3 c0Right = calculatePoint(roll, RollDirection::RIGHT);
	FloatingType psi0Right = -psi0Left;

	//Current position is c0 in the frame centered and rotated around roll = 0
	FloatingType psiFrameLeft = heading - psi0Left;
	FloatingType psiFrameRight = heading - psi0Right;

	frameLeft_.setYaw(psiFrameLeft);
	frameRight_.setYaw(psiFrameRight);

	frameLeft_.setOrigin(pos - frameLeft_.toInertialFramePosition(c0Left));
	frameRight_.setOrigin(pos - frameRight_.toInertialFramePosition(c0Right));

	wind_ = wind.velocity;
	//Calculate Endpoints
	yawrateOrbit_ = params.g() / velocity_ * std::tan(params.rollMax());
	endpointRight_ = calculatePoint(params.rollMax(), RollDirection::RIGHT);
	endpointRight_[2] = data.position.z();
	yawEndRight_ = calculateYaw(params.rollMax(), RollDirection::RIGHT);

	endpointLeft_ = calculatePoint(-params.rollMax(), RollDirection::LEFT);
	endpointLeft_[2] = data.position.z();
	yawEndLeft_ = calculateYaw(-params.rollMax(), RollDirection::LEFT);

	return true;
}

std::vector<Vector3>
ConstRollRateModel::getCriticalPoints(const Edge& edge, RollDirection dir)
{
	Lock lock(queryMutex_);
	std::vector<Vector3> result;

	auto yaw = calculateYaw(edge.yaw);
	for (const auto& it : calculateRoll(yaw, dir)) //TODO: calculate Roll is not really correct anymore
	{
		result.push_back(calculatePoint(it, dir));
	}

	auto end = std::atan2(edge.normal.y(), edge.normal.x());

	for (const auto& it : getCriticalPointsOrbit(edge.yaw, end, dir))
	{
		result.push_back(it);
	}
//	Vector3 normal(-edge.normal.x(), -edge.normal.y(), 0);
//	if (dir == RollDirection::LEFT)
//		result.push_back(centerOrbitLeft_ + normal * radiusOrbit_);
//	else
//		result.push_back(centerOrbitRight_ + normal * radiusOrbit_);

	return result;

}

Vector3
ConstRollRateModel::calculatePoint(FloatingType roll, RollDirection dir)
{
	acb_set_d(query_, pow(cos(roll), 2));
	FloatingType sign = roll > 0 ? 1 : -1;
	if (dir == RollDirection::LEFT)
	{
		acb_hypgeom_beta_lower(queryRes_, aLeft_, b_, query_, 0, params.precision());
		auto B = toVector(queryRes_);

		return frameLeft_.toInertialFramePosition(-(betaCompleteLeft_ - B) * factor_ * sign)
				+ getWindDisplacement((-roll + currentRoll_) / params.rollRate());
	}

	acb_hypgeom_beta_lower(queryRes_, aRight_, b_, query_, 0, params.precision());
	auto B = toVector(queryRes_);

	return frameRight_.toInertialFramePosition((betaCompleteRight_ - B) * factor_ * sign)
			+ getWindDisplacement((roll - currentRoll_) / params.rollRate());
}

FloatingType
ConstRollRateModel::calculateYaw(FloatingType roll, RollDirection dir)
{
	FloatingType psi = -params.g() / (velocity_ * params.rollRate()) * log(cos(roll));
	if (dir == RollDirection::LEFT)
	{
		return frameLeft_.toInertialFrameRotation(Vector3(0, 0, psi)).z();
	}
	return frameRight_.toInertialFrameRotation(Vector3(0, 0, -psi)).z();
}

std::vector<FloatingType>
ConstRollRateModel::calculateRoll(FloatingType yaw, RollDirection dir)
{
	std::vector<FloatingType> result;
	FloatingType yawCenter;
	FloatingType rollRate;
	FloatingType min, max;
	if (dir == RollDirection::LEFT)
	{
		yawCenter = frameLeft_.fromFrameRotation(InertialFrame(), Vector3(0, 0, yaw)).z();
		rollRate = -params.rollRate();
		min = -params.rollMax();
		max = currentRoll_;
		if (yawCenter < 0)
			yawCenter += M_PI;
	}
	else
	{
		yawCenter = frameRight_.fromFrameRotation(InertialFrame(), Vector3(0, 0, yaw)).z();
		rollRate = params.rollRate();
		min = currentRoll_;
		max = params.rollMax();
		if (yawCenter > 0)
			yawCenter -= M_PI;
	}

	FloatingType roll = acos(exp(rollRate * velocity_ / params.g() * yawCenter));
	FloatingType roll2 = acos(exp(rollRate * velocity_ / params.g() * (yawCenter + M_PI)));

	if (roll >= min && roll <= max)
		result.push_back(roll);
	if (roll2 >= min && roll2 <= max)
		result.push_back(roll2);

	if (-roll >= min && -roll <= max)
		result.push_back(-roll);
	if (-roll2 >= min && -roll2 <= max)
		result.push_back(-roll2);

	return result;
}

Vector3
ConstRollRateModel::toVector(const acb_t& complex)
{
	FloatingType x = arf_get_d(arb_midref(acb_realref(complex)), ARF_RND_NEAR);
	FloatingType y = arf_get_d(arb_midref(acb_imagref(complex)), ARF_RND_NEAR);

	return Vector3(x, y, 0);
}

FloatingType
ConstRollRateModel::getTimeFromYawOrbit(FloatingType yaw, RollDirection dir)
{
	if (dir == RollDirection::RIGHT)
	{
		auto yawDiff = yaw - yawEndRight_;
		if (yawDiff > 0)
			yawDiff -= 2 * M_PI;
		return -yawDiff / yawrateOrbit_;
	}
	else
	{
		auto yawDiff = yaw - yawEndLeft_;
		if (yawDiff < 0)
			yawDiff += 2 * M_PI;
		return yawDiff / yawrateOrbit_;
	}
}

Vector3
ConstRollRateModel::getPositionOrbit(FloatingType time, RollDirection dir)
{
	return Vector3();
}

Vector3
ConstRollRateModel::getPositionCurve(FloatingType time, RollDirection dir)
{
	return Vector3();
}

Vector3
ConstRollRateModel::getWindDisplacement(FloatingType time)
{
	return time * wind_;
}

std::vector<Vector3>
ConstRollRateModel::getCriticalPointsOrbit(FloatingType course, FloatingType end, RollDirection dir)
{
	auto course2 = boundAngleRad(course + M_PI);
	std::vector<Vector3> result;

	if (dir == RollDirection::LEFT)
	{

		auto startingCourse = calculateCourse(yawEndLeft_);
		auto diff = end - startingCourse;
		if (diff < 0)
			diff += 2 * M_PI;

		auto diffCourse = course - startingCourse;
		if (diffCourse < 0)
			diffCourse += 2 * M_PI;

		if (diffCourse < diff)
		{
			auto t = getTimeFromYawOrbit(calculateYaw(course), dir);
			result.push_back(calculatePointOrbit(t, dir));
		}

		auto diffCourse2 = course2 - startingCourse;
		if (diffCourse2 < 0)
			diffCourse2 += 2 * M_PI;

		if (diffCourse2 < diff)
		{
			auto t = getTimeFromYawOrbit(calculateYaw(course2), dir);
			result.push_back(calculatePointOrbit(t, dir));
		}

	}
	else
	{
		auto startingCourse = calculateCourse(yawEndRight_);
		auto diff = end - startingCourse;
		if (diff > 0)
			diff -= 2 * M_PI;

		auto diffCourse = course - startingCourse;
		if (diffCourse > 0)
			diffCourse -= 2 * M_PI;

		if (diffCourse > diff)
		{
			auto t = getTimeFromYawOrbit(calculateYaw(course), dir);
			result.push_back(calculatePointOrbit(t, dir));
		}

		auto diffCourse2 = course2 - startingCourse;
		if (diffCourse2 > 0)
			diffCourse2 -= 2 * M_PI;

		if (diffCourse2 > diff)
		{
			auto t = getTimeFromYawOrbit(calculateYaw(course2), dir);
			result.push_back(calculatePointOrbit(t, dir));
		}

	}

	return result;

}

FloatingType
ConstRollRateModel::calculateYaw(FloatingType course)
{
	return std::acos(
			(wind_.y() - wind_.x() * std::tan(course))
					/ (velocity_ * std::sqrt(std::pow(std::tan(course), 2) + 1)))
			- std::atan2(std::cos(course), std::sin(course));
}

FloatingType
ConstRollRateModel::calculateCourse(FloatingType yaw)
{
	return std::atan2(std::sin(yaw) * velocity_ + wind_.y(), std::cos(yaw) * velocity_ + wind_.x());
}

Vector3
ConstRollRateModel::calculatePointOrbit(FloatingType time, RollDirection dir)
{

	using namespace std::complex_literals;
	using Complex = std::complex<FloatingType>;

	FloatingType psi0;
	FloatingType psiDot;
	Vector3 startingPoint;

	if (dir == RollDirection::LEFT)
	{
		psi0 = yawEndLeft_;
		psiDot = yawrateOrbit_;
		startingPoint = endpointLeft_;
	}
	else
	{
		psi0 = yawEndRight_;
		psiDot = -yawrateOrbit_;
		startingPoint = endpointRight_;
	}

	Complex point = velocity_ / psiDot * std::exp(1i * (psi0 - M_PI / 2))
			* (std::exp(1i * psiDot * time) - 1.);

	Vector3 position(std::real(point), std::imag(point), 0);

	return startingPoint + position + getWindDisplacement(time);

}
