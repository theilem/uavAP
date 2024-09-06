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
 * PathSection.h
 *
 *  Created on: Jun 23, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CONTROL_GLOBALPLANNER_IPATHSECTION_H_
#define UAVAP_CONTROL_GLOBALPLANNER_IPATHSECTION_H_

#include "cpsCore/Utilities/LinearAlgebra.h"

struct SensorData;

enum class PathSectionType
{
    UNKNOWN = 0, CURVE, ORBIT, LINE, SPLINE
};

struct IPathSection
{
    virtual
    ~IPathSection() = default;

    // virtual void
    // updatePosition(const Vector3& pos) = 0;

    virtual void
    updateSensorData(const SensorData& data) = 0;

    virtual bool
    inTransition() const = 0;

    virtual Vector3
    getPositionDeviation() const = 0;

    virtual Vector3
    getDirection() const = 0;

    virtual FloatingType
    getSlope() const = 0;

    virtual FloatingType
    getCurvature() const = 0;

    virtual Vector3
    getEndPoint() const = 0;

    virtual FloatingType
    getVelocity() const = 0;
};

#endif /* UAVAP_CONTROL_GLOBALPLANNER_IPATHSECTION_H_ */
