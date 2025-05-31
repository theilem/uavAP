//
// Created by Mirco Theile on 29/5/25.
//

#ifndef EXTERNALPLANNER_H
#define EXTERNALPLANNER_H

#include <cpsCore/cps_object>

#include "cpsCore/Utilities/Scheduler/Event.h"

class DataPresentation;
class Packet;
class IScheduler;
class ILocalPlanner;
class IMissionPlanner;
class ApproachPlanner;
class IPC;

struct ExternalPlannerParams
{
    template <typename Configurator>
    void
    configure(Configurator& c)
    {
    }
};

class ExternalPlanner : public ConfigurableObject<ExternalPlannerParams>,
                        public AggregatableObject<IPC, ApproachPlanner, IMissionPlanner, ILocalPlanner, IScheduler, DataPresentation>,
                        public IRunnableObject
{
public:
    static constexpr auto typeId = "external_planner";
    ExternalPlanner() = default;
    ~ExternalPlanner() override = default;
    bool
    run(RunStage stage) override;
private:
    void
    onExternalPacket(const Packet& packet);
    void
    extenalTimeout();
    Event timeOutEvent_;
};
#endif //EXTERNALPLANNER_H
