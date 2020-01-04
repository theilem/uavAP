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
 * MissionControlPrint.h
 *
 *  Created on: Jan 15, 2018
 *      Author: uav
 */

#ifndef UAVAP_MISSIONCONTROL_MISSIONCONTROLPRINT_H_
#define UAVAP_MISSIONCONTROL_MISSIONCONTROLPRINT_H_
#include "uavAP/MissionControl/GlobalPlanner/PathSections/CubicSpline.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Curve.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Line.h"
#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"
#include <memory>
#include <ostream>

std::ostream&
operator<<(std::ostream& os, std::shared_ptr<IPathSection>& ps);

std::ostream&
operator<<(std::ostream& os, const Trajectory& traj);

std::ostream&
operator<<(std::ostream& io, const CubicSpline& s);

std::ostream&
operator<<(std::ostream& io, const Curve& s);

std::ostream&
operator<<(std::ostream& io, const Line& s);

std::ostream&
operator<<(std::ostream& io, const Orbit& s);

#endif /* UAVAP_MISSIONCONTROL_MISSIONCONTROLPRINT_H_ */
