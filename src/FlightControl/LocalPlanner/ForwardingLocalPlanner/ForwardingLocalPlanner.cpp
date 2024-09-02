//
// Created by Mirco Theile on 1/9/24.
//

#include "uavAP/FlightControl/LocalPlanner/ForwardingLocalPlanner/ForwardingLocalPlanner.h"

#include <cpsCore/Utilities/IPC/IPC.h>


bool
ForwardingLocalPlanner::run(RunStage stage)
{
    switch (stage)
    {
    case RunStage::INIT:
        {
            if (!checkIsSetAll())
            {
                CPSLOG_ERROR << "ForwardingLocalPlanner: missing dependencies";
                return true;
            }
            auto ipc = get<IPC>();
            trajectoryPublisher_ = ipc->publishPackets(params.ipcTarget());
            break;
        }
    default:
        break;
    }
    return false;
}

void
ForwardingLocalPlanner::setTrajectory(const Trajectory& traj)
{
    auto dp = get<DataPresentation>();
    auto packet = dp->serialize(traj);
    trajectory_ = traj;
    trajectoryPublisher_.publish(packet);
}

Trajectory
ForwardingLocalPlanner::getTrajectory() const
{
    return trajectory_;
}
