/*
 * Curve.h
 *
 *  Created on: Jun 23, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CONTROL_GLOBALPLANNER_CURVE_H_
#define UAVAP_CONTROL_GLOBALPLANNER_CURVE_H_

#include "uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Orbit.h"
#include <iostream>

struct Curve : Orbit
{
    Curve() = default;

    Curve(const Orbit& orbit, const Vector3& ep) :
        Orbit(orbit)
    {
        auto radiusVec = (ep - center).normalized();
        endPoint = center + radiusVec * radius;
        Vector3 normal{0., 0., static_cast<FloatingType>(orbitDirection == OrbitDirection::CW ? -1 : 1)};
        endPointDirection = normal.cross(radiusVec);
    }

    Curve(const Vector3& center, OrbitDirection direction, const Vector3& ep, FloatingType vel) :
        Orbit(center, direction, (ep - center).norm(), vel)
    {
        auto radiusVec = (ep - center).normalized();
        endPoint = center + radiusVec * radius;
        Vector3 normal{0., 0., static_cast<FloatingType>(orbitDirection == OrbitDirection::CW ? -1 : 1)};
        endPointDirection = normal.cross(radiusVec);
    }

    void
    updateSensorData(const SensorData& data) override
    {
        Orbit::updateSensorData(data);
        if (!wasInQuadrantFour)
        {
            bool behindEndPoint = (currentPosition - endPoint).dot(endPointDirection) < 0;
            bool onSideOfEndPoint = (currentPosition - center).dot(endPoint - center) > 0;
            wasInQuadrantFour = behindEndPoint && onSideOfEndPoint;
        }
    }

    bool
    inTransition() const override
    {
        return (currentPosition - endPoint).dot(endPointDirection) > 0 && wasInQuadrantFour;
    }

    std::optional<Vector3>
    getEndPoint() const override
    {
        return endPoint;
    }

    std::optional<Vector3>
    getEndDirection() const override
    {
        return endPointDirection;
    }

    std::string
    getDescription(bool currentState) const override
    {
        std::stringstream ss;
        ss << "Curve: center: " << center.transpose() << ", radius: " << radius
            << ", direction: " << (orbitDirection == OrbitDirection::CW ? "CW" : "CCW")
            << ", endPoint: " << endPoint.transpose()
            << ", endPointDirection: " << endPointDirection.transpose()
            << ", velocity: " << velocity;
        return ss.str();
    }

    Vector3 endPoint;
    Vector3 endPointDirection;
    bool wasInQuadrantFour{false}; // Used to determine if the curve was in the fourth quadrant
};

namespace dp
{
    template <class Archive, typename>
    void
    serialize(Archive& ar, Curve& t)
    {
        ar & static_cast<Orbit&>(t);

        ar & t.endPoint;
        ar & t.endPointDirection;
        ar & t.wasInQuadrantFour;
    }
}

#endif /* UAVAP_CONTROL_GLOBALPLANNER_CURVE_H_ */
