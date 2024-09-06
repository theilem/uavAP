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
 * Polygon.h
 *
 *  Created on: Aug 14, 2018
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_POLYGON_H_
#define UAVAP_MISSIONCONTROL_POLYGON_H_
#include <uavAP/Core/Rectanguloid.h>
#include <cpsCore/Utilities/LinearAlgebra.h>
#include <cpsCore/Configuration/Configuration.hpp>

struct Edge
{
	Vector2 offset;
	Vector2 direction;
	Vector2 normal; //Pointing inside

	FloatingType yaw; // [-pi, pi)

	//Create Edge from two points that are in counter clock wise order (left is inside)
	static Edge
	fromPoints(const Vector2& p1, const Vector2& p2);

	/**
	 * @brief Calculate distance to Edge. Returns distance as negative if query point lies behind Edge.
	 */
	FloatingType
	getDistance(const Vector2& query) const;

	/**
	 * @brief Calculate absolute distance to Edge. Returns only positive values.
	 */
	FloatingType
	getDistanceAbs(const Vector2& query) const;
};

class Polygon
{
public:

	Polygon&
	fromRectangle(const Rectanguloid& rect);

	bool
	configure(const Configuration& config);

	const std::vector<Edge>&
	getEdges() const;

private:

	std::vector<Edge> edges_;

};

#endif /* UAVAP_MISSIONCONTROL_POLYGON_H_ */
