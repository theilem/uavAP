/*
 * ApproachPlanner.cpp
 *
 *  Created on: Dec 16, 2017
 *      Author: mircot
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
ApproachPlanner::setMission(const Mission& mission)
{
    if (mission.waypoints().empty())
    {
        CPSLOG_ERROR << "Mission does not contain Waypoints. Ignore.";
        return;
    }

    // expecting mission with single waypoint (end point)
    mission_ = mission;

    // infinite should return false for the approach
    bool infinite = mission.infinite();

    Trajectory traj;
    PathSections pathSections;

    const auto& wp = mission.waypoints();
    // calculating orbit
    IPathSection partial_orbit;          // (point - radius) should always be on circumference
    partial_orbit = std::make_shared<PartialOrbit>(wp[0].location()-Vector3(50,0,0), Vector3(0, 0, 1), 50, 50);

    // calculating helix
    IPathSection helix;
    helix = std::make_shared<Helix>(wp[0].location()-Vector3(50,0,0), Vector3(0, 0, 1), 50, 50);
    if (infinite && params.naturalSplines())
    {
        // return path section list
        pathSections = createNaturalSplines(mission);
    }
    else
    {
        // returns path section list
        pathSections = createCatmulRomSplines(mission);
    }

    // adding helix then orbit to traj
    pathSections.push_back(helix);
    pathSections.push_back(partial_orbit);

    traj = Trajectory(pathSections, mission.infinite());
    
    auto lp = get<ILocalPlanner>();
    lp->setTrajectory(traj);
}

Mission
ApproachPlanner::getMission() const
{
    return mission_;
}

PathSections
ApproachPlanner::createNaturalSplines(const Mission& mission)
{
    const auto& wp = mission.waypoints();
    uint8_t n = params.inclusionLength() * 2;
    if (n == 0)
        n = static_cast<uint8_t>(wp.size());

    Matrix3 A, B, K, L, Rn;
    A << 1, 1, 1, 1, 2, 3, 0, 1, 3;

    B << 0, 0, 0, -1, 0, 0, 0, -1, 0;

    K << 3, -2, 1, -1, 3, -2, 1, -1, 1;

    L << -2, 1, 0, 3, -2, 0, -1, 1, 0;

    auto Lpow = L;
    //TODO Fix stupidness
    for (uint8_t i = 0; i < n - 1; ++i)
    {
        Lpow = Lpow * L;
    }

    Rn = (A + B * Lpow).inverse();

    std::vector<Matrix3> P;

    for (uint8_t j = 0; j < n; ++j)
    {
        Matrix3 temp;
        temp.setZero();
        if (j == n - 1)
            temp.row(0) = wp[0].location() - wp[j].location();
        else
            temp.row(0) = wp[j + 1].location() - wp[j].location();

        P.emplace_back(Rn * temp);
    }

    auto C = P;
    for (size_t i = 1; i < n; ++i)
    {
        for (size_t j = 0; j < n; ++j)
        {
            P[j] = L * P[j];
            auto k = (j + i) % (n - 1);
            C[k] += P[j];
            std::cout << C[j] << std::endl << std::endl;
        }
    }

    PathSections traj;
    for (size_t j = 0; j < n; ++j)
    {
        FloatingType velocity = (!wp[j].velocity()) ? mission.velocity() : *wp[j].velocity();
        // just need to set waypoint's altitude to the current altitude (.z)
        auto spline = std::make_shared<CubicSpline>(wp[j].location(), C[j].row(0), C[j].row(1),
                                                    C[j].row(2), velocity);
        traj.push_back(spline);
    }

    return traj;
}

PathSections
ApproachPlanner::createCatmulRomSplines(const Mission& mission)
{
    CPSLOG_DEBUG << "Create Catmull Rom Splines";
    // Need to set the waypoint altitude equal to current altitude somewhere in here...
    const auto& wp = mission.waypoints();
    bool infinite = mission.infinite();

    PathSections traj;

    Eigen::Matrix<FloatingType, 4, 4> tauMat;
    tauMat << 0, 1, 0, 0,
        -params.tau(), 0, params.tau(), 0,
        2 * params.tau(), params.tau() - 3, 3 - 2 * params.tau(), -params.tau(),
        -params.tau(), 2 - params.tau(), params.tau() - 2, params.tau();
    Eigen::Matrix<FloatingType, 4, 3> pointMat;
    Eigen::Matrix<FloatingType, 4, 3> approachMat;
    if (wp.size() == 1)
        infinite = false;
    else
    {
        if (infinite)
            pointMat.row(0) = wp.back().location().transpose();
        else
            pointMat.row(0) = wp.front().location().transpose();
        pointMat.row(1) = wp.front().location().transpose();
        pointMat.row(2) = wp[1].location().transpose();
    }

    for (auto it = wp.begin(); it != wp.end(); ++it)
    {
        if (it->direction())
            pointMat.row(0) = pointMat.row(2) - it->direction()->transpose();
        else
        {
            if (params.smoothenZ())
            {
                const auto& col = pointMat.col(2);
                FloatingType incline1 = col[2] - col[0];
                FloatingType incline2 = col[2] - col[1];
                FloatingType incline3 = col[1] - col[0];
                FloatingType minIncline = 0;
                if (fabs(incline1) < fabs(incline2))
                {
                    if (fabs(incline3) < fabs(incline1))
                        minIncline = incline3;
                    else
                        minIncline = incline1;
                }
                else
                {
                    if (fabs(incline3) < fabs(incline2))
                        minIncline = incline3;
                    else
                        minIncline = incline2;
                }

                pointMat.col(2)[0] = col[2] - minIncline;
            }
        }

        //		if (populateApproach)
        //		{
        //			approachMat.bottomRows(3) = pointMat.topRows(3);
        //			populateApproach = false;
        //		}

        auto nextIt = it + 1;
        if (nextIt == wp.end())
        {
            if (!infinite)
            {
                FloatingType velocity = (!it->velocity()) ? mission.velocity() : *it->velocity();
                traj.push_back(
                    std::make_shared<Orbit>(it->location(), Vector3::UnitZ(), params.orbitRadius(),
                                            velocity));
                break;
            }
            nextIt = wp.begin();
        }
        auto next = nextIt + 1;
        if (next == wp.end())
        {
            if (!infinite)
                next = nextIt;
            else
                next = wp.begin();
        }

        if (nextIt->direction())
            pointMat.row(3) = pointMat.row(1) + nextIt->direction()->transpose();
        else
        {
            pointMat.row(3) = next->location();
            if (params.smoothenZ())
            {
                const auto& col = pointMat.col(2);
                FloatingType incline1 = col[3] - col[1];
                FloatingType incline2 = col[3] - col[2];
                FloatingType incline3 = col[2] - col[1];
                FloatingType minIncline = 0;
                if (fabs(incline1) < fabs(incline2))
                {
                    if (fabs(incline3) < fabs(incline1))
                        minIncline = incline3;
                    else
                        minIncline = incline1;
                }
                else
                {
                    if (fabs(incline3) < fabs(incline2))
                        minIncline = incline3;
                    else
                        minIncline = incline2;
                }

                pointMat.col(2)[3] = col[1] + minIncline;
            }
        }

        Eigen::Matrix<FloatingType, 3, 4> C = (tauMat * pointMat).transpose();
        FloatingType velocity = (!it->velocity()) ? mission.velocity() : *it->velocity();
        auto spline = std::make_shared<CubicSpline>(C.col(0), C.col(1), C.col(2), C.col(3),
                                                    velocity);
        traj.push_back(spline);

        pointMat.topRows(3) = pointMat.bottomRows(3);

        pointMat.row(2) = next->location();
    }

    // Trajectory result(traj, mission.infinite());

    //	if (mission.initialPosition())
    //	{
    //		Waypoint init = *mission.initialPosition();
    //		approachMat.row(1) = init.location().transpose();
    //		if (init.direction())
    //			approachMat.row(0) = approachMat.row(2)
    //								 - (init.direction()->transpose() * mission.velocity());
    //		else
    //			approachMat.row(0) = approachMat.row(1);
    //
    //		Eigen::Matrix<FloatingType, 3, 4> C = (tauMat * approachMat).transpose();
    //		auto spline = std::make_shared<CubicSpline>(C.col(0), C.col(1), C.col(2), C.col(3),
    //													mission.velocity());
    //		result.approachSection = spline;
    //	}

    return traj;
}

Optional<Mission>
ApproachPlanner::missionRequest(const DataRequest& request)
{
    if (request == DataRequest::MISSION)
        return mission_;
    return std::nullopt;
}
