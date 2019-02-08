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
 * LinearAlgebra.cpp
 *
 *  Created on: Jun 26, 2017
 *      Author: mircot
 */

#include <iostream>

#include "uavAP/Core/LinearAlgebraProto.h"

Vector3
toVector(const PositionENU& pos)
{
	return Vector3(pos.east(), pos.north(), pos.up());
}

Vector3
rotate3Drad(const Vector3& vector, const AttitudeEuler& attitude)
{
	return Eigen::AngleAxisd(attitude.roll(), Vector3::UnitX())
			* Eigen::AngleAxisd(attitude.pitch(), Vector3::UnitY())
			* Eigen::AngleAxisd(attitude.yaw(), Vector3::UnitZ()) * vector;
}
