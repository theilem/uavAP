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
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Orbit.h"

// todo make this an Orbit
// fixed. keeping override on inTransition() and getSlope(). kept new function checkElevate().
struct Helix : public Orbit
{
public:

	Helix() = default;

	Helix(const Vector3& center, const Vector3& normal, FloatingType radius, FloatingType vel,
              Direction orientation, FloatingType slope) :
			Orbit(center, normal, radius, vel, orientation), slope_(slope), elevate_(true)
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

	// Q (updateSensorData()): do I need to change center to a temporaryCenter_ with the same z as current position?

    bool
	inTransition() const override
	{
		// Don't want to stay forever.. once altitude is close enough to center altitude can return true
		// Todo fix with threshold
        // now only checking if absolute value of difference is within the threshold.
        // shouldn't matter if above or below.
        FloatingType threshold = 10; // insert tau
		FloatingType elevationDifference = std::abs(center_.z() - currentPosition_.z());

        if (elevationDifference < threshold)
        {
            return true;
        }
		//Else stay in loop while you continue to move towards goal altitude
		return false;
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

    FloatingType slope_;
	bool elevate_;

};

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, Helix& t)
{
	ar & static_cast<Orbit&>(t);
	ar & t.slope_;
	ar & t.elevate_;
}
}

#endif /* UAVAP_CONTROL_GLOBALPLANNER_HELIX_H_ */