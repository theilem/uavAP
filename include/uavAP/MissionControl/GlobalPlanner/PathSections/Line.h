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
 * Line.h
 *
 *  Created on: Jun 23, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CONTROL_GLOBALPLANNER_LINE_H_
#define UAVAP_CONTROL_GLOBALPLANNER_LINE_H_

#include "uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h"
#include <iostream>
#include <uavAP/Core/SensorData.h>

struct Line: public IPathSection, public EigenLine
{

	Line() :
			velocity_(0)
	{
	}

	Line(const EigenLine& line, const Vector3& end, FloatingType vel) :
			EigenLine(line), endPoint_(end), currentPosition_(0, 0, 0), velocity_(vel)
	{
	}

	void
	updateSensorData(const SensorData& data) override
	{
		currentPosition_ = data.position;
	}

	bool
	inTransition() const override
	{
		return (currentPosition_ - endPoint_).dot(direction()) > 0;
	}

	Vector3
	getPositionDeviation() const override
	{
		return projection(currentPosition_) - currentPosition_;
	}

	Vector3
	getDirection() const override
	{
		return direction();
	}

	FloatingType
	getSlope() const override
	{
		return direction().z();
	}

	FloatingType
	getCurvature() const override
	{
		return 0;
	}

	Vector3
	getEndPoint() const override
	{
		return endPoint_;
	}

	void
	setEndPoint(const Vector3& end)
	{
		endPoint_ = end;
	}

	FloatingType
	getVelocity() const override
	{
		return velocity_;
	}

	Vector3 endPoint_;
	Vector3 currentPosition_;
	FloatingType velocity_;

};

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, Line& t)
{
	ar & static_cast<EigenLine&>(t);

	ar & t.endPoint_;
	ar & t.currentPosition_;
	ar & t.velocity_;
}
}

#endif /* UAVAP_CONTROL_GLOBALPLANNER_LINE_H_ */
