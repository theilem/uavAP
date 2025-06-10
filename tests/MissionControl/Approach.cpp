//
// Created by Mirco Theile on 28/5/25.
//

#include <cpsCore/Utilities/Test/TestInfo.h>
#include "uavAP/MissionControl/GlobalPlanner/ApproachPlanner/ApproachPlanner.h"

TEST_CASE("Approach Planner")
{
    ApproachPlanner approachPlanner;
    REQUIRE(approachPlanner.run(RunStage::INIT) == false);
    REQUIRE(approachPlanner.run(RunStage::NORMAL) == false);
    REQUIRE(approachPlanner.run(RunStage::FINAL) == false);

    Vector3 initPosition(0.0, 0.0, 100.0);
    Vector3 initDirection(1.0, 0.0, 0.0);
    Pose initPose(initPosition, initDirection);

    Vector3 goalPosition(10.0, 0.0, 50.0);
    Vector3 goalDirection(-1.0, 0.0, 0.0);
    Pose goalPose(goalPosition, goalDirection);

    auto path = approachPlanner.getApproach(initPose, goalPose);
}