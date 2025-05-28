/*
 * PathSection.h
 *
 *  Created on: Jun 23, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CONTROL_GLOBALPLANNER_IPATHSECTION_H_
#define UAVAP_CONTROL_GLOBALPLANNER_IPATHSECTION_H_

#include <optional>
#include "cpsCore/Utilities/LinearAlgebra.h"

struct SensorData;

enum class PathSectionType
{
    UNKNOWN = 0, CURVE, SPIRAL, ORBIT, LINE, SPLINE, QUARTIC_SPLINE
};

struct IPathSection
{
    virtual
    ~IPathSection() = default;

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

    virtual std::optional<Vector3>
    getEndPoint() const = 0;

    virtual std::optional<Vector3>
    getEndDirection() const = 0;

    virtual std::optional<Vector3>
    getStartingPoint() const = 0;

    virtual std::optional<Vector3>
    getStartingDirection() const = 0;

    virtual FloatingType
    getVelocity() const = 0;

    virtual std::string
    getDescription(bool currentState) const = 0;
};

#endif /* UAVAP_CONTROL_GLOBALPLANNER_IPATHSECTION_H_ */
