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
 * MissionControlPrint.cpp
 *
 *  Created on: Jan 15, 2018
 *      Author: uav
 */

#include "uavAP/MissionControl/MissionControlPrint.h"

std::ostream&
operator<<(std::ostream& os, std::shared_ptr<IPathSection>& ps)
{
	if (ps)
	{
		if (auto curve = std::dynamic_pointer_cast<Curve>(ps))
		{
			os << "Curve: " << std::endl;
			os << *curve;
		}
		else if (auto orbit = std::dynamic_pointer_cast<Orbit>(ps))
		{
			os << "Orbit: " << std::endl;
			os << *orbit;
		}
		else if (auto line = std::dynamic_pointer_cast<Line>(ps))
		{
			os << "Line: " << std::endl;
			os << *line;
		}
		else if (auto spline = std::dynamic_pointer_cast<CubicSpline>(ps))
		{
			os << "CubicSpline: " << std::endl;
			os << *spline;
		}
		else
		{
			os << "Unknown" << std::endl;
		}
	}
	return os;
}

std::ostream&
operator<<(std::ostream& os, const Trajectory& traj)
{
	for (auto it : traj.pathSections)
	{
		os << it;
	}
	return os;
}

std::ostream&
operator<<(std::ostream& io, const CubicSpline& s)
{
	io << "c0: " << s.c0_;
	io << "c1: " << s.c1_;
	io << "c2: " << s.c2_;
	io << "c3: " << s.c3_;
	io << "Velocity: " << s.velocity_;
	return io;
}

std::ostream&
operator<<(std::ostream& io, const Curve& s)
{
	//TODO
	return io;
}

std::ostream&
operator<<(std::ostream& io, const Line& s)
{
	//TODO
	return io;
}

std::ostream&
operator<<(std::ostream& io, const Orbit& s)
{
	//TODO
	return io;
}
