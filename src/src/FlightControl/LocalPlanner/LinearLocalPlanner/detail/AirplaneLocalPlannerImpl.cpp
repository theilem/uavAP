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
 * AirplaneLocalPlannerImpl.cpp
 *
 *  Created on: Aug 18, 2017
 *      Author: mircot
 */
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/detail/AirplaneLocalPlannerImpl.h"
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/detail/LocalPlannerParams.h"
#include "uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/LinearLocalPlanner.h"

#include <boost/variant.hpp>
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"

AirplaneLocalPlannerImpl::AirplaneLocalPlannerImpl() :
		headingTarget_(0)
{
}

bool
AirplaneLocalPlannerImpl::configure(const boost::property_tree::ptree& config)
{
	return PropertyMapper::configure(params_,config);
}

bool
AirplaneLocalPlannerImpl::tuneParams(const LocalPlannerParams& params)
{
	if (!params.has_linear_params())
	{
		return false;
	}
	if (!params.linear_params().has_airplane_params())
	{
		return false;
	}
	params_.CopyFrom(params.linear_params().airplane_params());
	return true;
}

ControllerTarget
AirplaneLocalPlannerImpl::evaluate(const Vector3& position, double heading,
		std::shared_ptr<IPathSection> section)
{
	ControllerTarget controllerTarget;

	controllerTarget.velocity[0] = section->getVelocity();
	auto positionDeviation = section->getPositionDeviation();

	// Climb Rate
	double climbRate = controllerTarget.velocity[0] * section->getSlope()
			+ params_.k_altitude() * positionDeviation.z();

	//Climb angle
	controllerTarget.climbAngle = sin(climbRate / section->getVelocity());

	// Heading
	Vector2 directionTarget_ = params_.k_heading() * positionDeviation.head(2)
			+ section->getDirection().head(2).normalized();
	headingTarget_ = headingFromENU(directionTarget_);

	double headingError = boundAngleRad(headingTarget_ - heading);

	// Yaw Rate
	controllerTarget.yawRate = controllerTarget.velocity[0] * section->getCurvature()
			+ params_.k_yaw_rate() * headingError;
	return controllerTarget;
}

LocalPlannerStatus
AirplaneLocalPlannerImpl::getStatus()
{
	LocalPlannerStatus status;
	auto stat = status.mutable_linear_status()->mutable_airplane_status();
	stat->set_heading_target(headingTarget_);
	return status;
}
