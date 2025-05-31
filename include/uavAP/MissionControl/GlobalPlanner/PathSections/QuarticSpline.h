/*
 * Spline.h
 *
 *  Created on: Dec 12, 2024
 *  Author: Mirco Theile (mirco.theile@tum.de)
 */

#ifndef UAVAP_MISSIONCONTROL_GLOBALPLANNER_PATHSECTIONS_QUARTICSPLINE_H_
#define UAVAP_MISSIONCONTROL_GLOBALPLANNER_PATHSECTIONS_QUARTICSPLINE_H_

#include <cmath>

#include "uavAP/Core/SensorData.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h"

struct QuarticSpline : IPathSection
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    QuarticSpline() = default;

    QuarticSpline(const QuarticSpline& other) = default;

    QuarticSpline(const Vector3& c0, const Vector3& c1, const Vector3& c2, const Vector3& c3, const Vector3& c4,
                  FloatingType velocity) :
        c0(c0), c1(c1), c2(c2), c3(c3), c4(c4), velocity(velocity)
    {
    }

    Vector3
    positionFromU(FloatingType u) const
    {
        return c0 + c1 * u + c2 * std::pow(u, 2) + c3 * std::pow(u, 3) + c4 * std::pow(u, 4);
    }

    void
    updateSensorData(const SensorData& data) override
    {
        currentPosition = data.position;

        int maxIter = 10;
        FloatingType convThreshold = 0.0001;
        auto posClosest = positionFromU(closestU);

        FloatingType distClosest = (posClosest - currentPosition).squaredNorm();
        FloatingType distZero = (c0 - currentPosition).squaredNorm();
        //Newton method to find closest point
        FloatingType u;
        if (distClosest < distZero)
            u = closestU;
        else
            u = 0;

        for (int i = 0; i < maxIter; ++i)
        {
            Vector3 p = positionFromU(u) - currentPosition;
            Vector3 p_prime = c1 + 2 * c2 * u + 3 * c3 * pow(u, 2) + 4 * c4 * pow(u, 3);
            Vector3 p_2prime = 2 * c2 + 6 * c3 * u + 12 * c4 * pow(u, 2);

            FloatingType grad = (p.dot(p_prime)) / (p_prime.dot(p_prime) + p.dot(p_2prime));

            if (std::isnan(grad))
                break;

            u = u - grad;
            if (fabs(grad) < convThreshold)
                break;
        }

        closestU = std::clamp(u, static_cast<FloatingType>(0), static_cast<FloatingType>(1));
    }

    bool
    inTransition() const override
    {
        if (closestU >= 1) //In transition if u for the closest point out of definition for current spline
        {
            return true;
        }
        return false;
    }

    Vector3
    getPositionDeviation() const override
    {
        return positionFromU(closestU) - currentPosition;
    }

    Vector3
    getDirection() const override
    {
        return (c1 + 2 * c2 * closestU + 3 * c3 * pow(closestU, 2) + 4 * c4 * pow(closestU, 3)).normalized();
    }

    FloatingType
    getSlope() const override
    {
        return getDirection().z();
    }

    FloatingType
    getCurvature() const override
    {
        Vector3 dp_du = c1 + 2 * c2 * closestU + 3 * c3 * pow(closestU, 2) + 4 * c4 * pow(closestU, 3);
        Vector3 dp_ddu = 2 * c2 + 6 * c3 * closestU + 12 * c4 * pow(closestU, 2);

        FloatingType dPsi_du = (dp_ddu[1] * dp_du[0] - dp_ddu[0] * dp_du[1])
            / pow((pow(dp_du[0], 2) + pow(dp_du[1], 2)), 3.0 / 2.0);

        return dPsi_du;
    }

    std::optional<Vector3>
    getEndPoint() const override
    {
        return c0 + c1 + c2 + c3 + c4;
    }

    std::optional<Vector3>
    getStartingPoint() const override
    {
        return c0;
    }

    std::optional<Vector3>
    getEndDirection() const override
    {
        return (c1 + 2 * c2 + 3 * c3 + 4 * c4).normalized(); //Derivative at u = 1
    }

    std::optional<Vector3>
    getStartingDirection() const override
    {
        return c1.normalized(); //Derivative at u = 0
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
        ss << "Quartic Spline: c0: " << c0.transpose() << ", c1: " << c1.transpose()
            << ", c2: " << c2.transpose() << ", c3: " << c3.transpose() << ", c4: " << c4.transpose() << ", velocity: "
            << velocity;
        if (currentState)
            ss << ", closestU: " << closestU
                << ", currentPosition: " << currentPosition.transpose();
        return ss.str();
    }

    FloatingType closestU{0.0};
    Vector3 currentPosition;

    //Quartic Spline: p(u) = c0 + c1 * u + c2 * u^2 + c3 * u^3 + c4 * u^4
    Vector3 c0;
    Vector3 c1;
    Vector3 c2;
    Vector3 c3;
    Vector3 c4;

    FloatingType velocity{};
};

namespace dp
{
    template <class Archive, typename Type>
    inline void
    serialize(Archive& ar, QuarticSpline& t)
    {
        ar & t.c0;
        ar & t.c1;
        ar & t.c2;
        ar & t.c3;
        ar & t.c4;
        ar & t.closestU;
        ar & t.velocity;
    }
}

#endif /* UAVAP_MISSIONCONTROL_GLOBALPLANNER_PATHSECTIONS_QUARTICSPLINE_H_ */
