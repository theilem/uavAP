/*
 * ApproachPlanner.cpp
 *
 *  Created on: May 14, 2025
 *      Author: acmeighan, jjponnia
 */
#include "uavAP/MissionControl/GlobalPlanner/PathSections/CubicSpline.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Orbit.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/PartialOrbit.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Helix.h"
#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"
#include "uavAP/MissionControl/GlobalPlanner/ApproachPlanner/ApproachPlanner.h"
#include "uavAP/FlightControl/LocalPlanner/ILocalPlanner.h"
#include "uavAP/Core/DataHandling/DataHandling.h"
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <cpsCore/Utilities/IPC/IPC.h>


bool
ApproachPlanner::run(RunStage stage)
{
    switch (stage)
    {
    case RunStage::INIT:
        {
            if (!checkIsSet<ILocalPlanner>())
            {
                return true;
            }
            break;
        }
    case RunStage::NORMAL:
        {
            if (auto dh = get<DataHandling>())
            {
                dh->addTriggeredStatusFunction<Mission, DataRequest>(
                    std::bind(&ApproachPlanner::missionRequest, this,
                              std::placeholders::_1), Content::MISSION, Content::REQUEST_DATA);
                dh->addConfig(this, Content::SPLINE_GLOBAL_PLANNER_PARAMS);
            }
            break;
        }
    default:
        break;
    }
    return false;
}

void
ApproachPlanner::setMission(const Waypoint& wp0, const Waypoint& wp1)
{
    //if (mission.waypoints().empty())
    //{
    //    CPSLOG_ERROR << "Mission does not contain Waypoints. Ignore.";
    //    return;
    //}

    //const auto& wp = mission.waypoints();

    // calculating orbit
    auto partial_orbit = std::make_shared<PartialOrbit>(wp1.location()-Vector3(50,0,0), Vector3(0, 0, 1), 50, 50, Direction::COUNTER_CLOCKWISE, wp1.location());

    // calculating helix
    // Jonathan edit - don't know what the slope or direction should be
    auto helix = std::make_shared<Helix>(wp1.location()-Vector3(50,0,0), Vector3(0, 0, 1), 50, 50, Direction::COUNTER_CLOCKWISE, 0.1);

    // returns path section list
    trajectory = createCatmulRomSplines(wp0, wp1);

    // adding helix then orbit to traj
    trajectory.pathSections.push_back(helix);
    trajectory.pathSections.push_back(partial_orbit);
    
    auto lp = get<ILocalPlanner>();
    lp->setTrajectory(trajectory);
}

Mission
ApproachPlanner::getMission() const
{
    return mission_;
}

Trajectory
ApproachPlanner::createCatmulRomSplines(const Waypoint& wp0, const Waypoint& wp1)
{

    // Assume that mission only contains two waypoints with a direction each
    CPSLOG_DEBUG << "Create Catmull Rom Splines";
    // Need to set the waypoint altitude equal to current altitude somewhere in here...

    Eigen::Matrix<FloatingType, 4, 4> tauMat;
    tauMat << 0, 1, 0, 0,
        -params.tau(), 0, params.tau(), 0,
        2 * params.tau(), params.tau() - 3, 3 - 2 * params.tau(), -params.tau(),
        -params.tau(), 2 - params.tau(), params.tau() - 2, params.tau();
    Eigen::Matrix<FloatingType, 4, 3> pointMat;
    Eigen::Matrix<FloatingType, 4, 3> approachMat;

    // initializing pointMat
    pointMat.row(0) = wp1.location().transpose() - wp0.direction()->transpose();
    pointMat.row(1) = wp0.location().transpose();
    pointMat.row(2) = wp1.location().transpose();
    pointMat.row(3) = wp0.location().transpose() + wp1.direction()->transpose();

    // ensuring that there is no change in the z-direction. this elevation can be saved for helix
    // setting all other altitudes equal to the altitude (z) of wp0
    pointMat.col(2)[0] = pointMat.col(2)[1];
    pointMat.col(2)[2] = pointMat.col(2)[1];
    pointMat.col(2)[3] = pointMat.col(2)[1];

    Eigen::Matrix<FloatingType, 3, 4> C = (tauMat * pointMat).transpose();
    // this velocity line may be incorrect
    FloatingType velocity = (!wp0.velocity()) ? mission.velocity() : *wp0.velocity();
    auto spline = std::make_shared<CubicSpline>(C.col(0), C.col(1), C.col(2), C.col(3),
                                                    velocity);
    Trajectory trajectory;
    trajectory.pathSections.push_back(spline);

    return trajectory;
}

Optional<Mission>
ApproachPlanner::missionRequest(const DataRequest& request)
{
    if (request == DataRequest::MISSION)
        return mission_;
    return std::nullopt;
}
