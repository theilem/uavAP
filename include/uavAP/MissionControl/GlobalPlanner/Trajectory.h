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
 * Trajectory.h
 *
 *  Created on: Jun 23, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_GLOBALPLANNER_TRAJECTORY_H_
#define UAVAP_FLIGHTCONTROL_GLOBALPLANNER_TRAJECTORY_H_

#include <uavAP/MissionControl/GlobalPlanner/PathSections/CubicSpline.h>
#include <uavAP/MissionControl/GlobalPlanner/PathSections/Line.h>
#include <uavAP/MissionControl/GlobalPlanner/PathSections/Orbit.h>
#include <uavAP/MissionControl/GlobalPlanner/PathSections/Curve.h>
#include "uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h"

#include <cpsCore/Utilities/DataPresentation/detail/Split.h>

#include <vector>
#include <memory>
#include <iostream>

using PathSections = std::vector<std::shared_ptr<IPathSection>>;
using PathSectionIterator = std::vector<std::shared_ptr<IPathSection>>::iterator;

struct Trajectory
{
	PathSections pathSections;
	std::shared_ptr<IPathSection> approachSection;
	bool infinite;

	Trajectory() :
			infinite(false)
	{
	}

	Trajectory(const PathSections& path, bool inf) :
			pathSections(path), infinite(inf)
	{
	}

	Trajectory(const PathSections& path, std::shared_ptr<IPathSection> approach, bool inf) :
			pathSections(path), approachSection(approach), infinite(inf)
	{
	}

};



namespace dp
{
template<class Archive, typename Type>
void
store(Archive & ar, const std::shared_ptr<IPathSection> & t)
{
	bool notNull = t != nullptr;
	ar << notNull;
	if (notNull)
	{
		if (auto curve = std::dynamic_pointer_cast<Curve>(t))
		{
			ar << PathSectionType::CURVE;
			ar << *curve;
		}
		else if (auto orbit = std::dynamic_pointer_cast<Orbit>(t))
		{
			ar << PathSectionType::ORBIT;
			ar << *orbit;
		}
		else if (auto line = std::dynamic_pointer_cast<Line>(t))
		{
			ar << PathSectionType::LINE;
			ar << *line;
		}
		else if (auto spline = std::dynamic_pointer_cast<CubicSpline>(t))
		{
			ar << PathSectionType::SPLINE;
			ar << *spline;
		}
		else
		{
			ar << PathSectionType::UNKNOWN;
		}
	}
}

template<class Archive, typename Type>
void
load(Archive & ar, std::shared_ptr<IPathSection> & t)
{
	bool notNull = false;
	ar >> notNull;
	if (notNull)
	{
		PathSectionType type;
		ar >> type;
		if (type == PathSectionType::CURVE)
		{
			auto curve = std::make_shared<Curve>();
			ar >> *curve;
			t = curve;
		}
		else if (type == PathSectionType::ORBIT)
		{
			auto orbit = std::make_shared<Orbit>();
			ar >> *orbit;
			t = orbit;
		}
		else if (type == PathSectionType::LINE)
		{
			auto line = std::make_shared<Line>();
			ar >> *line;
			t = line;
		}
		else if (type == PathSectionType::SPLINE)
		{
			auto spline = std::make_shared<CubicSpline>();
			ar >> *spline;
			t = spline;
		}
		else
		{
			t = nullptr;
		}
	}
	else
	{
		t = nullptr;
	}
}

template<class Archive, typename Type>
inline void
serialize(Archive& ar, std::shared_ptr<IPathSection>& t)
{
	split(ar, t);
}

template<class Archive, typename Type>
inline void
serialize(Archive& ar, Trajectory& t)
{
	ar & t.pathSections;
	ar & t.approachSection;
	ar & t.infinite;
}
} //namespace dp

#endif /* UAVAP_FLIGHTCONTROL_GLOBALPLANNER_TRAJECTORY_H_ */
