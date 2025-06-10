//
// Created by Mirco Theile on 29/5/25.
//

#include "uavAP/MissionControl/ExternalPlanner/ExternalPlanner.h"

#include "cpsCore/Utilities/IDC/Header/Hash.h"
#include "uavAP/FlightControl/LocalPlanner/ILocalPlanner.h"
#include "uavAP/MissionControl/GlobalPlanner/ApproachPlanner/approach.h"
#include "uavAP/MissionControl/GlobalPlanner/ApproachPlanner/ApproachPlanner.h"
#include "uavAP/MissionControl/MissionPlanner/IMissionPlanner.h"
#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"
#include "cpsCore/Utilities/IPC/IPC.h"
#include "cpsCore/Utilities/DataPresentation/DataPresentation.h"

bool
ExternalPlanner::run(RunStage stage)
{
    switch (stage)
    {
    case RunStage::INIT:
        {
            if (!checkIsSetAll())
            {
                CPSLOG_ERROR << "ExternalPlanner: Dependency missing";
                return true;
            }
            break;
        }
    case RunStage::NORMAL:
        {
            auto ipc = get<IPC>();
            ipc->subscribeOnPackets("external_to_mission_control", [this](const Packet& packet)
            {
                onExternalPacket(packet);
            });
        }
    default:
        break;
    }
    return false;
}

void
ExternalPlanner::onExternalPacket(const Packet& packet)
{
    Packet p = packet;
    auto dp = get<DataPresentation>();
    auto contentHeader = dp->extractHeader<Hash>(p);
    if (contentHeader == "approach")
    {
        auto pose = dp->deserialize<Pose>(p);
        auto approachPlanner = get<ApproachPlanner>();
        auto approach = approachPlanner->getApproach(pose);
        auto lp = get<ILocalPlanner>();
        Trajectory traj(approach, false);
        lp->setTrajectory(traj);
    }
    else if (contentHeader == "trajectory")
    {
        auto traj = dp->deserialize<Trajectory>(p);
        auto lp = get<ILocalPlanner>();
        lp->setTrajectory(traj);
    }
    else
    {
        CPSLOG_ERROR << "ExternalPlanner: Unknown content header: " << contentHeader.hashValue;
    }
}

void
ExternalPlanner::extenalTimeout()
{
    auto mp = get<IMissionPlanner>();
    mp->missionRequest("default_mission");
}
