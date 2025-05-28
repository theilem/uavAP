//
// Created by Mirco Theile on 27/5/25.
//

#include "uavAP/MissionControl/GlobalPlanner/ApproachPlanner/approach.h"

#include "uavAP/MissionControl/GlobalPlanner/Geometry.h"

PathSections
approach(const Pose& start, const Pose& end, FloatingType velocity, FloatingType slope, FloatingType radius)
{
    auto endOrbit1 = poseToOrbit(end, OrbitDirection::CCW, radius, velocity);
    auto endOrbit2 = poseToOrbit(end, OrbitDirection::CW, radius, velocity);

    auto traj1 = approach(start, endOrbit1, velocity, slope, radius);
    auto traj2 = approach(start, endOrbit2, velocity, slope, radius);

    if (traj1.empty() && traj2.empty())
    {
        CPSLOG_ERROR << "Failed to compute approach trajectory. This should not happen.";
        return PathSections();
    }
    PathSections bestTraj;
    std::shared_ptr<Orbit> finalOrbit;
    if (traj1.empty())
    {
        bestTraj = traj2;
        finalOrbit = endOrbit2;
    }
    else if (traj2.empty())
    {
        bestTraj = traj1;
        finalOrbit = endOrbit1;
    }
    else
    {
        // get the trajectory with the shortest straight line path
        auto line1 = std::dynamic_pointer_cast<Line>(traj1.back());
        auto line2 = std::dynamic_pointer_cast<Line>(traj2.back());
        if (!line1 || !line2)
        {
            CPSLOG_ERROR << "Failed to compute approach trajectory. This should not happen.";
            return PathSections();
        }
        FloatingType dist1 = (*line1->getEndPoint() - *line1->getStartingPoint()).norm();
        FloatingType dist2 = (*line2->getEndPoint() - *line2->getStartingPoint()).norm();
        if (dist1 < dist2)
        {
            bestTraj = traj1;
            finalOrbit = endOrbit1;
        }
        else
        {
            bestTraj = traj2;
            finalOrbit = endOrbit2;
        }
    }
    // Add the final curve to the trajectory
    auto finalCurve = std::make_shared<Curve>(*finalOrbit, end.first);
    bestTraj.push_back(finalCurve);
    return bestTraj;
}

PathSections
approach(const Pose& start, std::shared_ptr<Orbit> end, FloatingType velocity, FloatingType slope, FloatingType radius)
{
    Pose startAtAltitude{start.first, start.second};
    startAtAltitude.first.z() = end->getCenter().z();
    auto orbit1 = poseToOrbit(startAtAltitude, OrbitDirection::CCW, radius, velocity);
    auto orbit2 = poseToOrbit(startAtAltitude, OrbitDirection::CW, radius, velocity);

    std::optional<Tangent> tangent1;
    std::optional<Tangent> tangent2;
    if (orbit1->orbitDirection == end->orbitDirection)
    {
        tangent1 = computeExternalTangentPoint(
            orbit1->getCenter().head<2>(), orbit1->getRadius(),
            end->getCenter().head<2>(), end->getRadius(),
            orbit1->orbitDirection);
        tangent2 = computeInternalTangentPoint(
            orbit2->getCenter().head<2>(), orbit2->getRadius(),
            end->getCenter().head<2>(), end->getRadius(),
            orbit2->orbitDirection);
    }
    else
    {
        tangent1 = computeInternalTangentPoint(
            orbit1->getCenter().head<2>(), orbit1->getRadius(),
            end->getCenter().head<2>(), end->getRadius(),
            orbit1->orbitDirection);
        tangent2 = computeExternalTangentPoint(
            orbit2->getCenter().head<2>(), orbit2->getRadius(),
            end->getCenter().head<2>(), end->getRadius(),
            orbit2->orbitDirection);
    }

    if (!tangent1 && !tangent2)
    {
        CPSLOG_ERROR << "Failed to compute tangent points for approach. This should not happen.";
        return PathSections();
    }
    Tangent bestTangent;
    std::shared_ptr<Orbit> bestOrbit;
    if (tangent1 && !tangent2)
    {
        bestTangent = *tangent1;
        bestOrbit = orbit1;
    }
    else if (!tangent1 && tangent2)
    {
        bestTangent = *tangent2;
        bestOrbit = orbit2;
    }
    else
    {
        // get the tangent with the shortest straight line path
        FloatingType dist1 = (tangent1->pointA - tangent1->pointB).norm();
        FloatingType dist2 = (tangent2->pointA - tangent2->pointB).norm();
        if (dist1 < dist2)
        {
            bestTangent = *tangent1;
            bestOrbit = orbit1;
        }
        else
        {
            bestTangent = *tangent2;
            bestOrbit = orbit2;
        }
    }
    PathSections traj;
    auto spiral = std::make_shared<Spiral>(*bestOrbit, end->getCenter().z(), slope);
    Vector3 transition(bestTangent.pointA.x(), bestTangent.pointA.y(), end->getCenter().z());
    auto curve = std::make_shared<Curve>(*bestOrbit, transition);
    auto line = std::make_shared<Line>(transition,
                                       Vector3(bestTangent.pointB.x(), bestTangent.pointB.y(), end->getCenter().z()),
                                       end->getVelocity());
    return {spiral, curve, line};

}


std::shared_ptr<Orbit>
poseToOrbit(const Pose& pose, OrbitDirection direction, FloatingType radius, FloatingType vel)
{
    Vector3 radiusVec(-pose.second.y(), pose.second.x(), 0);
    if (direction == OrbitDirection::CW)
        radiusVec = -radiusVec;
    radiusVec.normalize();
    auto center = pose.first + radiusVec * radius;
    return std::make_shared<Orbit>(center, direction, radius, vel);
}
