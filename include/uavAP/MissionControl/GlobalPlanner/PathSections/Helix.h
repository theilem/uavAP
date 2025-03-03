////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2024 Indiana University
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
 * Helix.h
 *
 *  Created on: Sept 3, 2024
 *      Author: acmeighan
 */

#ifndef UAVAP_CONTROL_GLOBALPLANNER_HELIX_H_
#define UAVAP_CONTROL_GLOBALPLANNER_HELIX_H_

#include "uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h"
#include <iostream>
#include <uavAP/Core/SensorData.h>

// todo make this an Orbit
struct Helix : public IPathSection
{
public:

	Helix() :
			radius_(0), velocity_(0)
	{
	}

	Helix(const Vector3& center, const Vector3& normal, const SensorData& data, FloatingType radius, FloatingType slope,
		  FloatingType vel) :
			center_(center), normal_(normal), initialPosition_(data), radius_(radius), slope_(slope), velocity_(vel),
			currentPosition_(0, 0, 0), elevate_(true)
	{
	}

	void
	checkElevate(const Vector3& position)
	{
		if (position.z() < center_.z())
		{
			elevate_ = true;
		}
		else
		{
			elevate_ = false;
		}
	}

	void
	updateSensorData(const SensorData& data) override
	{
		currentPosition_ = data.position;
		checkElevate(currentPosition_);
		//Calculate current radius vector to target position as it is used several times
		Vector3 projection = EigenHyperplane(normal_, center_).projection(currentPosition_);
		radiusVector_ = (projection - center_).normalized() * radius_;
	}


	bool
	inTransition() const override
	{
		// Don't want to stay forever.. once altitude is close enough to center altitude can return true
		// Todo fix with threshold
		if (elevate_)
		{
			if (center_.z() < currentPosition_.z())
			{
				return true;
			}
		}
		else // This triggers when elevate_ is false
		{
			if (center_.z() > currentPosition_.z()) // Adjust condition as needed
			{
				return true;
			}
		}

		//Else stay in loop while you continue to move towards goal altitude
		return false;
	}

	Vector3
	getPositionDeviation() const override
	{
		Vector3 positionDeviation_ = radiusVector_ + center_ - currentPosition_;
		positionDeviation_.z = 0;
		return positionDeviation_;
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

	FloatingType
	getRadius() const
	{
		return radius_;
	}

	FloatingType
	getSlope() const override
	{
		if (elevate_)
		{
			return slope_;
		}
		else
		{
			return -slope_;
		}
	}

	FloatingType
	getCurvature() const override
	{
		FloatingType curv = 1 / radius_;
		return normal_.z() < 0 ? -curv : curv;
	}

	Vector3
	getEndPoint() const override
	{
		return center_;
	}

	FloatingType
	getVelocity() const override
	{
		return velocity_;
	}

	Vector3 center_;
	Vector3 normal_;
	FloatingType radius_;
	Vector3 radiusVector_;
	FloatingType slope_;
	FloatingType velocity_;

	Vector3 currentPosition_;
	bool elevate_;
};

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, Helix& t)
{
	ar & t.center_;
	ar & t.normal_;
	ar & t.radius_;
	ar & t.radiusVector_;
	ar & t.slope_;
	ar & t.velocity_;
	ar & t.currentPosition_;
	ar & t.elevate_;
}
}

#endif /* UAVAP_CONTROL_GLOBALPLANNER_HELIX_H_ */