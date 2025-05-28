//
// Created by Mirco Theile on 26/5/25.
//

#ifndef SPIRAL_H
#define SPIRAL_H
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Orbit.h"


struct Spiral : Orbit
{
    Spiral() = default;

    Spiral(const Orbit& orbit, FloatingType targetAltitude, FloatingType slope) :
        Orbit(orbit), targetAltitude(targetAltitude), slope(slope)
    {
    }

    Spiral(const Vector3& center, OrbitDirection direction, FloatingType targetAltitude, FloatingType slope,
           FloatingType radius,
           FloatingType vel) :
        Orbit(center, direction, radius, vel), targetAltitude(targetAltitude), slope(slope)
    {
    }

    void
    updateSensorData(const SensorData& data) override
    {
        Orbit::updateSensorData(data);
        if (!isInitialized)
        {
            isInitialized = true;
            wasBelowTarget = data.position.z() < targetAltitude;
        }
    }

    FloatingType
    getSlope() const override
    {
        return wasBelowTarget ? std::abs(slope) : -std::abs(slope);
    }

    bool
    inTransition() const override
    {
        // Transition if the current position is below the target altitude and it was below the target altitude before
        // or if the current position is above the target altitude and it was below the target altitude before
        return (currentPosition.z() < targetAltitude && !wasBelowTarget) ||
            (currentPosition.z() > targetAltitude && wasBelowTarget);
    }

    Vector3
    getPositionDeviation() const override
    {
        auto dev = Orbit::getPositionDeviation();
        dev.z() = 0;
        return dev;
    }

    std::string
    getDescription(bool currentState) const override
    {
        std::stringstream ss;
        ss << "Spiral: center: " << center.transpose() << ", radius: " << radius
            << ", direction: " << (orbitDirection == OrbitDirection::CW ? "CW" : "CCW")
            << ", target altitude: " << targetAltitude
            << ", slope: " << slope
            << ", velocity: " << velocity;
        if (currentState)
            ss << ", currentPosition: " << currentPosition.transpose()
                << ", radiusVector: " << radiusVector.transpose()
                << ", wasBelowTarget: " << std::boolalpha << wasBelowTarget
                << ", isInitialized: " << isInitialized;
        return ss.str();
    }

    FloatingType targetAltitude{150.0f};
    FloatingType slope{0.0f};
    bool wasBelowTarget{false};
    bool isInitialized{false};
};

namespace dp
{
    template <class Archive, typename>
    void
    serialize(Archive& ar, Spiral& t)
    {
        ar & static_cast<Orbit&>(t);
        ar & t.targetAltitude;
        ar & t.slope;
        ar & t.wasBelowTarget;
        ar & t.isInitialized;
    }
} // namespace dp
#endif //SPIRAL_H
