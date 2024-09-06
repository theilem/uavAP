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
 * Polygon.cpp
 *
 *  Created on: Aug 20, 2018
 *      Author: mircot
 */
#include <uavAP/MissionControl/Polygon.h>

Edge
Edge::fromPoints(const Vector2& p1, const Vector2& p2)
{
	Edge edge;
	edge.offset = p1;
	edge.direction = (p2 - p1).normalized();
	edge.normal = Vector2(-edge.direction[1], edge.direction[0]);
	edge.yaw = atan2(edge.direction.y(), edge.direction.x());
	return edge;
}

FloatingType
Edge::getDistance(const Vector2& query) const
{
	return normal.dot(query - offset);
}

FloatingType
Edge::getDistanceAbs(const Vector2& query) const
{
	return fabs(getDistance(query));
}

bool
Polygon::configure(const Configuration& config)
{
	return true;
}

Polygon&
Polygon::fromRectangle(const Rectanguloid& rect)
{
	Vector2 major = rotate2Drad(Vector2(rect.majorSideLength(), 0.0),
			rect.majorSideOrientation()) / 2;
	Vector2 minor = rotate2Drad(Vector2(0.0, rect.minorSideLength()),
			rect.majorSideOrientation()) / 2;

	Vector2 center(0, 0);
	center[0] = rect.center()[0];
	center[1] = rect.center()[1];

	edges_.push_back(Edge::fromPoints(center + major + minor, center - major + minor));
	edges_.push_back(Edge::fromPoints(center - major + minor, center - major - minor));
	edges_.push_back(Edge::fromPoints(center - major - minor, center + major - minor));
	edges_.push_back(Edge::fromPoints(center + major - minor, center + major + minor));

	return *this;
}

const std::vector<Edge>&
Polygon::getEdges() const
{
	return edges_;
}
