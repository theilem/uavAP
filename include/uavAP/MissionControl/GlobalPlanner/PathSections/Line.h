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
#include "uavAP/Core/SensorData.h"

struct Line : IPathSection, EigenLine
{
    Line() :
        velocity(20)
    {
    }

    Line(const Vector3& start, const Vector3& end, FloatingType vel) :
        EigenLine(start, (end - start).normalized()), endPoint(end), currentPosition(0, 0, 0), velocity(vel)
    {
    }

    Line(const EigenLine& line, const Vector3& end, FloatingType vel) :
        EigenLine(line), endPoint(end), currentPosition(0, 0, 0), velocity(vel)
    {
    }

    void
    updateSensorData(const SensorData& data) override
    {
        currentPosition = data.position;
    }

    bool
    inTransition() const override
    {
        return (currentPosition - endPoint).dot(direction()) > 0;
    }

    Vector3
    getPositionDeviation() const override
    {
        return projection(currentPosition) - currentPosition;
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

    std::optional<Vector3>
    getEndPoint() const override
    {
        return endPoint;
    }

    std::optional<Vector3>
    getEndDirection() const override
    {
        return direction();
    }

    std::optional<Vector3>
    getStartingDirection() const override
    {
        return direction();
    }

    std::optional<Vector3>
    getStartingPoint() const override
    {
        return origin();
    }

    void
    setEndPoint(const Vector3& end)
    {
        endPoint = end;
    }

    FloatingType
    getVelocity() const override
    {
        return velocity;
    }


    std::string
    getDescription(bool currentState) const override
    {
        std::stringstream ss;
        ss << "Line: start: " << origin().transpose() << ", end: " << endPoint.transpose()
            << ", direction: " << direction().transpose() << ", velocity: " << velocity;
        return ss.str();
    }

    Vector3 endPoint;
    Vector3 currentPosition;
    FloatingType velocity;
};

namespace dp
{
    template <class Archive, typename>
    void
    serialize(Archive& ar, Line& t)
    {
        ar & static_cast<EigenLine&>(t);

        ar & t.endPoint;
        ar & t.currentPosition;
        ar & t.velocity;
    }
}

#endif /* UAVAP_CONTROL_GLOBALPLANNER_LINE_H_ */
