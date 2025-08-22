//
// Created by Mirco Theile on 27/5/25.
//

#ifndef APPROACHPLANNER_H
#define APPROACHPLANNER_H

#include <cpsCore/cps_object>

#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"
#include "uavAP/MissionControl/GlobalPlanner/ApproachPlanner/approach.h"

class ILocalPlanner;
class ISensingIO;

struct ApproachPlannerParams
{
    Parameter<FloatingType> slope = {0.2, "slope", true};
    Parameter<FloatingType> radius = {100.0, "radius", true};
    Parameter<FloatingType> velocity = {20.0, "velocity", true};
    Parameter<FloatingType> endingStraightLength = {0.0, "ending_straight_length", true}; // Length of the straight line at the end of the approach
    template <typename Configurator>
    void
    configure(Configurator& c)
    {
        c & slope;
        c & radius;
        c & velocity;
        c & endingStraightLength;
    }
};

class ApproachPlanner : public ConfigurableObject<ApproachPlannerParams>,
                        public AggregatableObject<ISensingIO, ILocalPlanner>,
                        public IRunnableObject
{
public:
    static constexpr auto typeId = "approach_planner";
    ApproachPlanner() = default;
    ~ApproachPlanner() override = default;
    bool
    run(RunStage stage) override;
    PathSections
    getApproach(const Pose& start, const Pose& goal) const;
    PathSections
    getApproach(const Pose& start, std::shared_ptr<Orbit> goalOrbit) const;
    PathSections
    getApproach(const Pose& start, const PathSections& periodicPart) const;
    PathSections
    getApproach(const Pose& goal) const;
    PathSections
    getApproach(std::shared_ptr<Orbit> goalOrbit) const;
    PathSections
    getApproach(const PathSections& periodicPart) const;

};

#endif //APPROACHPLANNER_H
