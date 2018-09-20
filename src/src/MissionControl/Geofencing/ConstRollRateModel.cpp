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

ConstRollRateModel::ConstRollRateModel() :
		rollRate_(0), rollMax_(0), precision_(0), g_(9.81), currentRoll_(0), factor_(0), velocity_(
				0), radiusOrbit_(0)
{
	acb_init(aLeft_);
	acb_init(aRight_);
	acb_init(b_);
	acb_init(query_);
	acb_init(queryRes_);

	acb_set_d(b_, 0.5);
}

ConstRollRateModel::~ConstRollRateModel()
{
	acb_clear(aLeft_);
	acb_clear(aRight_);
	acb_clear(b_);
	acb_clear(query_);
	acb_clear(queryRes_);
}

std::shared_ptr<ConstRollRateModel>
ConstRollRateModel::create(const boost::property_tree::ptree& config)
{
	auto constRollRateModel = std::make_shared<ConstRollRateModel>();

	if (!constRollRateModel->configure(config))
	{
		APLOG_ERROR << "ConstRollRateModel: Failed to Load Config.";
	}

	return constRollRateModel;
}

bool
ConstRollRateModel::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	pm.add<double>("roll_rate", rollRate_, true);
	pm.add<double>("roll_max", rollMax_, true);
	pm.add<long int>("precision", precision_, true);
	pm.add<double>("g", g_, false);

	degToRadRef(rollRate_);
	degToRadRef(rollMax_);

	return pm.map();
}

void
ConstRollRateModel::notifyAggregationOnUpdate(const Aggregator& agg)
{
}

bool
ConstRollRateModel::updateModel(const SensorData& data)
{
	std::unique_lock<std::mutex> lock(queryMutex_, std::try_to_lock);
	if (!lock.owns_lock())
	{
		APLOG_DEBUG << "Data in use. Will update next time.";
		return false;
	}
	velocity_ = data.airSpeed;
	currentRoll_ = data.attitude.x();
	acb_set_d_d(aLeft_, 0.5, -g_ / (2 * velocity_ * rollRate_));
	acb_set_d_d(aRight_, 0.5, g_ / (2 * velocity_ * rollRate_));
	acb_one(query_);
	acb_hypgeom_beta_lower(queryRes_, aLeft_, b_, query_, 0, precision_);
	betaCompleteLeft_ = toVector(queryRes_);
	acb_hypgeom_beta_lower(queryRes_, aRight_, b_, query_, 0, precision_);
	betaCompleteRight_ = toVector(queryRes_);

	factor_ = velocity_ / (2 * rollRate_);

	//Calculate Centers
	//Left -> rollRate negative, rollMax negative
	const double& roll = currentRoll_;
	const double& heading = data.attitude[2];
	const Vector3& pos = data.position;

	//Reset frames

	frameLeft_.setOrigin(Vector3(0, 0, 0));
	frameLeft_.setYaw(0);
	frameRight_.setOrigin(Vector3(0, 0, 0));
	frameRight_.setYaw(0);

	Vector3 c0Left = calculatePoint(roll, RollDirection::LEFT);
	double psi0Left = calculateYaw(roll, RollDirection::LEFT);
	Vector3 c0Right = calculatePoint(roll, RollDirection::RIGHT);
	double psi0Right = -psi0Left;

	//Current position is c0 in the frame centered and rotated around roll = 0
	double psiFrameLeft = heading - psi0Left;
	double psiFrameRight = heading - psi0Right;

	frameLeft_.setYaw(psiFrameLeft);
	frameRight_.setYaw(psiFrameRight);

	frameLeft_.setOrigin(pos - frameLeft_.toInertialFramePosition(c0Left));
	frameRight_.setOrigin(pos - frameRight_.toInertialFramePosition(c0Right));

	//Calculate Orbit centers
	radiusOrbit_ = pow(velocity_, 2) / (tan(rollMax_) * g_);
	Vector3 endpoint = calculatePoint(rollMax_, RollDirection::RIGHT);
	double psimax = calculateYaw(rollMax_, RollDirection::RIGHT);

	centerOrbitRight_ = endpoint
			- Vector3(cos(psimax + M_PI / 2.0), sin(psimax + M_PI / 2.0), 0) * radiusOrbit_;
	centerOrbitRight_[2] = data.position.z();

	endpoint = calculatePoint(-rollMax_, RollDirection::LEFT); //TODO maybe shortcut due to symmetry
	psimax = calculateYaw(-rollMax_, RollDirection::LEFT);

	centerOrbitLeft_ = endpoint
			- Vector3(cos(psimax - M_PI / 2.0), sin(psimax - M_PI / 2.0), 0) * radiusOrbit_;
	centerOrbitLeft_[2] = data.position.z();

	return true;
}

std::vector<Vector3>
ConstRollRateModel::getCriticalPoints(const Edge& edge, RollDirection dir)
{
	std::unique_lock<std::mutex> lock(queryMutex_);
	std::vector<Vector3> result;
	for (const auto& it : calculateRoll(edge.yaw, dir))
	{
		result.push_back(calculatePoint(it, dir));
	}
	Vector3 normal(-edge.normal.x(), -edge.normal.y(), 0);
	if (dir == RollDirection::LEFT)
		result.push_back(centerOrbitLeft_ + normal * radiusOrbit_);
	else
		result.push_back(centerOrbitRight_ + normal * radiusOrbit_);

	return result;

}

Vector3
ConstRollRateModel::calculatePoint(double roll, RollDirection dir)
{
	acb_set_d(query_, pow(cos(roll), 2));
	double sign = roll > 0 ? 1 : -1;
	if (dir == RollDirection::LEFT)
	{
		acb_hypgeom_beta_lower(queryRes_, aLeft_, b_, query_, 0, precision_);
		auto B = toVector(queryRes_);

		return frameLeft_.toInertialFramePosition(-(betaCompleteLeft_ - B) * factor_ * sign);
	}

	acb_hypgeom_beta_lower(queryRes_, aRight_, b_, query_, 0, precision_);
	auto B = toVector(queryRes_);

	return frameRight_.toInertialFramePosition((betaCompleteRight_ - B) * factor_ * sign);
}

double
ConstRollRateModel::calculateYaw(double roll, RollDirection dir)
{
	double psi = -g_ / (velocity_ * rollRate_) * log(cos(roll));
	if (dir == RollDirection::LEFT)
	{
		return frameLeft_.toInertialFrameRotation(Vector3(0, 0, psi)).z();
	}
	return frameRight_.toInertialFrameRotation(Vector3(0, 0, -psi)).z();
}

std::vector<double>
ConstRollRateModel::calculateRoll(double yaw, RollDirection dir)
{
	std::vector<double> result;
	double yawCenter;
	double rollRate;
	double min, max;
	if (dir == RollDirection::LEFT)
	{
		yawCenter = frameLeft_.fromFrameRotation(InertialFrame(), Vector3(0, 0, yaw)).z();
		rollRate = -rollRate_;
		min = -rollMax_;
		max = currentRoll_;
		if (yawCenter < 0)
			yawCenter += M_PI;
	}
	else
	{
		yawCenter = frameRight_.fromFrameRotation(InertialFrame(), Vector3(0, 0, yaw)).z();
		rollRate = rollRate_;
		min = currentRoll_;
		max = rollMax_;
		if (yawCenter > 0)
			yawCenter -= M_PI;
	}

	double roll = acos(exp(rollRate * velocity_ / g_ * yawCenter));
	double roll2 = acos(exp(rollRate * velocity_ / g_ * (yawCenter + M_PI)));

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
	double x = arf_get_d(arb_midref(acb_realref(complex)), ARF_RND_NEAR);
	double y = arf_get_d(arb_midref(acb_imagref(complex)), ARF_RND_NEAR);

	return Vector3(x, y, 0);
}
