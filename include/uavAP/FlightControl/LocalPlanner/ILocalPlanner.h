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
 * ITrajectoryPlanner.h
 *
 *  Created on: Jun 2, 2017
 *      Author: mircot
 */

#ifndef UAVAP_FLIGHTCONTROL_LOCALPLANNER_ILOCALPLANNER_H_
#define UAVAP_FLIGHTCONTROL_LOCALPLANNER_ILOCALPLANNER_H_

#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"

class ILocalPlanner
{
public:

	static constexpr auto typeId = "local_planner";

	virtual
	~ILocalPlanner() = default;

	virtual void
	setTrajectory(const Trajectory& traj) = 0;

	virtual Trajectory
	getTrajectory() const = 0;

};

#endif /* UAVAP_FLIGHTCONTROL_LOCALPLANNER_ILOCALPLANNER_H_ */
