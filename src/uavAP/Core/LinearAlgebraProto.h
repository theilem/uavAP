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
/**
 *  @file         LinearAlgebra.h
 *  @author  Mirco Theile, mirco.theile@tum.de
 *  @date      23 June 2017
 *  @brief      Vector handling, conversions and rotations
 */

#ifndef UAVAP_CORE_LINEARALGEBRA_PROTO_H_
#define UAVAP_CORE_LINEARALGEBRA_PROTO_H_


#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/Core/protobuf/messages/Attitudes.pb.h"
#include "uavAP/Core/protobuf/messages/Positions.pb.h"

class PositionENU;

/**
 * @brief Convert protobuf ENUPosition into Eigen::Vector3d
 * @param position Position in ENU
 * @return Position as Eigen::Vector3d
 */
Vector3
toVector(const PositionENU& position);

/**
 * @brief Rotate 3D vector counter clockwise
 * @param vector Vector to be rotated
 * @param attitude Euler attitude angles in radians
 * @return Rotated vector
 */
Vector3
rotate3Drad(const Vector3& vector, const AttitudeEuler& attitude);


#endif /* UAVAP_CORE_LINEARALGEBRA_PROTO_H_ */
