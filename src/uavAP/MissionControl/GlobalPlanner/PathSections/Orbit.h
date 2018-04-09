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
 * Orbit.h
 *
 *  Created on: Jun 23, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CONTROL_GLOBALPLANNER_ORBIT_H_
#define UAVAP_CONTROL_GLOBALPLANNER_ORBIT_H_

#include "uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h"
#include <iostream>

struct Orbit: public IPathSection
{
public:

	Orbit():
		radius_(0),
		velocity_(0)
	{
	}

	Orbit(const Vector3& center, const Vector3& normal, double radius, double vel) :
			center_(center), normal_(normal), radius_(radius), velocity_(vel), currentPosition_(0, 0, 0)
	{
	}

	void
	updatePosition(const Vector3& pos) override
	{
		currentPosition_ = pos;
		//Calculate current radius vector to target position as it is used several times
		Vector3 projection = EigenHyperplane(normal_, center_).projection(currentPosition_);
		radiusVector_ = (projection - center_).normalized() * radius_;
	}

	bool
	inTransition() const override
	{
		//Stay forever
		return false;
	}

	Vector3
	getPositionDeviation() const override
	{
		return radiusVector_ + center_ - currentPosition_;
	}

	Vector3
	getDirection() const override
	{
		return normal_.cross(radiusVector_ / radius_);
	}

	Vector3
	getCenter() const
	{
		return center_;
	}

	double
	getRadius() const
	{
		return radius_;
	}

	double
	getSlope() const override
	{
		return getDirection().z();
	}

	double
	getCurvature() const override
	{
		double curv = 1 / radius_;
		return normal_.z() < 0 ? curv : -curv;
	}

	Vector3
	getEndPoint() const override
	{
		return center_;
	}

	double
	getVelocity() const override
	{
		return velocity_;
	}


	Vector3 center_;
	Vector3 normal_;
	double radius_;
	Vector3 radiusVector_;
	double velocity_;

	Vector3 currentPosition_;
};




#endif /* UAVAP_CONTROL_GLOBALPLANNER_ORBIT_H_ */
