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
 * HelicopterLocalPlannerImpl.cpp
 *
 *  Created on: Sep 1, 2017
 *      Author: simonyu
 */

#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/detail/HelicopterLocalPlannerImpl.h"
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/detail/LocalPlannerParams.h"
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/LinearLocalPlanner.h"

#include <boost/variant.hpp>
#include "uavAP/Core/Logging/APLogger.h"

bool
HelicopterLocalPlannerImpl::configure(const boost::property_tree::ptree& config)
{
	return params_.configure(config);
}

bool
HelicopterLocalPlannerImpl::tuneParams(const LocalPlannerParams& params)
{
	return false;
}

ControllerTarget
HelicopterLocalPlannerImpl::evaluate(const Vector3& position, double heading,
		std::shared_ptr<IPathSection> section)
{
	ControllerTarget controllerTarget;

	auto velocityPath = section->getVelocity();
	auto positionDeviation = section->getPositionDeviation();
	directionTarget_ = section->getDirection().head(2);

	Vector2 velocityTarget = directionTarget_ + params_.kVelocity_ * positionDeviation.head(2);
	Vector2 velocityTargetRotated = Rotation2(-heading) * velocityTarget.normalized();
	Vector2 velocityTargetScaled = velocityPath * velocityTargetRotated;

	controllerTarget.velocity.head(2) = velocityTargetScaled;
	controllerTarget.velocity.z() = velocityPath * section->getSlope()
			+ params_.kAltitude_ * positionDeviation.z();

	double headingTarget = (std::atan2(directionTarget_.y(), directionTarget_.x())
			+ params_.kHeading_ * std::atan2(velocityTarget.y(), velocityTarget.x()))
			/ (1 + params_.kHeading_);
	double headingError = headingTarget - heading;

	if (headingError > M_PI)
	{
		headingError -= 2 * M_PI;
	}
	else if (headingError < -M_PI)
	{
		headingError += 2 * M_PI;
	}

	controllerTarget.yawRate = params_.kYawRate_ * headingError;

	return controllerTarget;
}

LocalPlannerStatus
HelicopterLocalPlannerImpl::getStatus()
{
	LocalPlannerStatus status;
	//TODO Do stuff here
	return status;
}
