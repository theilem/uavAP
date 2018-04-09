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
#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h"

#include <vector>
#include <memory>
#include <iostream>

using PathSections = std::vector<std::shared_ptr<IPathSection>>;
using PathSectionIterator = std::vector<std::shared_ptr<IPathSection>>::iterator;

struct Trajectory
{
	PathSections pathSections;
	bool infinite;

	Trajectory() :
			infinite(false)
	{
	}

	Trajectory(const PathSections& path, bool inf) :
			pathSections(path),
			infinite(inf)
	{
	}

};


#endif /* UAVAP_FLIGHTCONTROL_GLOBALPLANNER_TRAJECTORY_H_ */
