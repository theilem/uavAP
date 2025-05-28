/*
 * Orbit.h
 *
 *  Created on: Jun 23, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CONTROL_GLOBALPLANNER_ORBIT_H_
#define UAVAP_CONTROL_GLOBALPLANNER_ORBIT_H_

#include "uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h"
#include <uavAP/Core/SensorData.h>

enum class OrbitDirection
{
    CW,
    CCW
};

struct Orbit : IPathSection
{
    Orbit() :
        radius(100), velocity(20)
    {
    }

    Orbit(const Vector3& center, OrbitDirection direction, FloatingType radius, FloatingType vel) :
        center(center), radius(radius), velocity(vel), currentPosition(0, 0, 0), orbitDirection(direction)
    {
    }

    void
    updateSensorData(const SensorData& data) override
    {
        currentPosition = data.position;
        //Calculate current radius vector to target position as it is used several times
        Vector3 projection = EigenHyperplane(Vector3(0, 0, 1), center).projection(currentPosition);
        radiusVector = (projection - center).normalized() * radius;
    }

    bool
    inTransition() const override
    {
        //Stay forever
        return false;
    }

    Vector3
    getPositionDeviation() const override
    {
        return radiusVector + center - currentPosition;
    }

    Vector3
    getDirection() const override
    {
        Vector3 normal{0., 0., static_cast<FloatingType>(orbitDirection == OrbitDirection::CW ? -1 : 1)};
        return normal.cross(radiusVector / radius);
    }

    Vector3
    getCenter() const
    {
        return center;
    }

    FloatingType
    getRadius() const
    {
        return radius;
    }

    FloatingType
    getSlope() const override
    {
        return getDirection().z();
    }

    FloatingType
    getCurvature() const override
    {
        FloatingType curv = 1. / radius;
        return orbitDirection == OrbitDirection::CW ? -curv : curv;
    }

    std::optional<Vector3>
    getEndPoint() const override
    {
        return std::nullopt;
    }

    std::optional<Vector3>
    getEndDirection() const override
    {
        return std::nullopt;
    }

    std::optional<Vector3>
    getStartingDirection() const override
    {
        return std::nullopt;
    }

    std::optional<Vector3>
    getStartingPoint() const override
    {
        return std::nullopt;
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
        ss << "Orbit: center: " << center.transpose() << ", radius: " << radius
            << ", direction: " << (orbitDirection == OrbitDirection::CW ? "CW" : "CCW")
            << ", velocity: " << velocity;
        if (currentState)
            ss << ", currentPosition: " << currentPosition.transpose()
                << ", radiusVector: " << radiusVector.transpose();
        return ss.str();
    }


    Vector3 center;
    FloatingType radius;
    Vector3 radiusVector;
    FloatingType velocity;

    Vector3 currentPosition;
    OrbitDirection orbitDirection{OrbitDirection::CCW};
};

namespace dp
{
    template <class Archive, typename>
    void
    serialize(Archive& ar, Orbit& t)
    {
        ar & t.center;
        ar & t.orbitDirection;
        ar & t.radius;
        ar & t.radiusVector;
        ar & t.velocity;
        ar & t.currentPosition;
    }
}

#endif /* UAVAP_CONTROL_GLOBALPLANNER_ORBIT_H_ */
