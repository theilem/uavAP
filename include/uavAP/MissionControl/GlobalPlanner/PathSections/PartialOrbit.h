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
 * PartialOrbit.h
 *
 *  Created on: Jun 23, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CONTROL_GLOBALPLANNER_PARTIALORBIT_H_
#define UAVAP_CONTROL_GLOBALPLANNER_PARTIALORBIT_H_

#include "uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h"
#include <iostream>
#include <uavAP/Core/SensorData.h>
#include <cmath>

struct PartialOrbit : public Orbit
{
public:

	PartialOrbit() = default;

	PartialOrbit(const Vector3& center, const Vector3& normal, FloatingType radius, FloatingType vel,
				 Direction orientation, const Vector3& goalPoint) :
			Orbit(center, normal, radius, vel, orientation), goalPoint_(goalPoint)
	{
	}

	bool
	inTransition() const override
	{
		double l2_norm = std::sqrt(std::pow(goalPoint_.x() - currentPosition_.x(), 2) +
								   std::pow(goalPoint_.y() - currentPosition_.y(), 2));
		if (l2_norm < radius_ / 20.0)
		{
			return true;
		}
		//Stay forever
		return false;
	}

	Vector3 goalPoint_;

};

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, PartialOrbit& t)
{
	ar & static_cast<Orbit&>(t);
	ar & t.goalPoint_;
}
}

#endif /* UAVAP_CONTROL_GLOBALPLANNER_PARTIALORBIT_H_ */
