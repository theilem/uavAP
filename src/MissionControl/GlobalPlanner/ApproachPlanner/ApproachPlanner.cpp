//
// Created by Mirco Theile on 27/5/25.
//

#include <utility>

#include "uavAP/MissionControl/GlobalPlanner/ApproachPlanner/ApproachPlanner.h"

#include "uavAP/FlightControl/SensingActuationIO/ISensingIO.h"

bool
ApproachPlanner::run(RunStage stage)
{
    switch (stage)
    {
    case RunStage::INIT:
        if (!isSet<ISensingIO>())
        {
            CPSLOG_DEBUG << "ApproachPlanner: SensingIO not set. Cannot approach from current position.";
        }
        if (!isSet<ILocalPlanner>())
        {
            CPSLOG_DEBUG << "ApproachPlanner: LocalPlanner not set. Cannot actively approach.";
        }
        break;
    case RunStage::NORMAL:
        break;
    case RunStage::FINAL:
        break;
    default:
        break;
    }
    return false;
}

PathSections
ApproachPlanner::getApproach(const Pose& start, const Pose& goal) const
{
    return approach(start, goal, params.velocity(), params.slope(), params.radius());
}

PathSections
ApproachPlanner::getApproach(const Pose& start, std::shared_ptr<Orbit> goalOrbit) const
{
    return approach(start, std::move(goalOrbit), params.velocity(), params.slope(), params.radius());
}

PathSections
ApproachPlanner::getApproach(const Pose& start, const PathSections& periodicPart) const
{
    const auto& initial = periodicPart.front();
    auto startingPoint = initial->getStartingPoint();
    auto startingDir = initial->getStartingDirection();
    if (startingPoint && startingDir)
        return getApproach(start, {*startingPoint, *startingDir});
    if (auto orbit = std::dynamic_pointer_cast<Orbit>(initial))
        return getApproach(start, orbit);
    CPSLOG_ERROR <<
        "Cannot compute approach trajectory. The first section of the periodic part is not a valid starting point.";
    return {};
}

PathSections
ApproachPlanner::getApproach(const Pose& goal) const
{
    auto sens = get<ISensingIO>();
    if (!sens)
    {
        CPSLOG_ERROR << "ApproachPlanner: SensingIO not set. Cannot approach from current position.";
        return {};
    }
    auto sd = sens->getSensorData();
    Pose currentPose{sd.position, sd.velocity.normalized()};
    return getApproach(currentPose, goal);
}

PathSections
ApproachPlanner::getApproach(std::shared_ptr<Orbit> goalOrbit) const
{
    auto sens = get<ISensingIO>();
    if (!sens)
    {
        CPSLOG_ERROR << "ApproachPlanner: SensingIO not set. Cannot approach from current position.";
        return {};
    }
    auto sd = sens->getSensorData();
    Pose currentPose{sd.position, sd.velocity.normalized()};
    return getApproach(currentPose, std::move(goalOrbit));
}

PathSections
ApproachPlanner::getApproach(const PathSections& periodicPart) const
{
    auto sens = get<ISensingIO>();
    if (!sens)
    {
        CPSLOG_ERROR << "ApproachPlanner: SensingIO not set. Cannot approach from current position.";
        return {};
    }
    auto sd = sens->getSensorData();
    Pose currentPose{sd.position, sd.velocity.normalized()};
    return getApproach(currentPose, periodicPart);
}
