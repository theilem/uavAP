//
// Created by Mirco Theile on 1/9/24.
//

#ifndef FORWARDINGLOCALPLANNER_H
#define FORWARDINGLOCALPLANNER_H

#include <cpsCore/cps_object>
#include <cpsCore/Utilities/IPC/Publisher.h>

#include "uavAP/FlightControl/LocalPlanner/ILocalPlanner.h"
#include "uavAP/FlightControl/LocalPlanner/ForwardingLocalPlanner/ForwardingLocalPlannerParams.h"


class DataPresentation;
class IPC;

class ForwardingLocalPlanner : public ILocalPlanner,
                               public AggregatableObject<IPC, DataPresentation>,
                               public ConfigurableObject<ForwardingLocalPlannerParams>,
                               public IRunnableObject
{
public:

    static constexpr TypeId typeId = "forwarding";

    ForwardingLocalPlanner() = default;

    ~ForwardingLocalPlanner() override = default;

    bool
    run(RunStage stage) override;

    void
    setTrajectory(const Trajectory& traj) override;

    Trajectory
    getTrajectory() const override;

private:

    Publisher<Packet> trajectoryPublisher_;
    Trajectory trajectory_;
};


#endif //FORWARDINGLOCALPLANNER_H
