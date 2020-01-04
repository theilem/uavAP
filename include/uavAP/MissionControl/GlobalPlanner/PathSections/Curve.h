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
 * Curve.h
 *
 *  Created on: Jun 23, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CONTROL_GLOBALPLANNER_CURVE_H_
#define UAVAP_CONTROL_GLOBALPLANNER_CURVE_H_

#include "uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Orbit.h"
#include <iostream>

struct Curve: public Orbit
{

	Curve()
	{
	}

	Curve(const Vector3& center, const Vector3& normal, const Vector3 endPoint, FloatingType vel) :
			Orbit(center, normal, (endPoint - center).norm(), vel), endPoint_(endPoint)
	{
		auto radius = (endPoint_ - center).normalized();
		endPointDirection_ = normal.cross(radius);
	}

	bool
	inTransition() const override
	{
		return (currentPosition_ - endPoint_).dot(endPointDirection_) > 0;
	}

	Vector3
	getEndPoint() const override
	{
		return endPoint_;
	}

	Vector3 endPoint_;
	Vector3 endPointDirection_;

};

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, Curve& t)
{
	ar & static_cast<Orbit&>(t);

	ar & t.endPoint_;
	ar & t.endPointDirection_;
}
}

#endif /* UAVAP_CONTROL_GLOBALPLANNER_CURVE_H_ */
