/*
 * FramesTest.cpp
 *
 *  Created on: May 15, 2025
 *      Author: acmeighan
 */

#include <cpsCore/Utilities/Test/TestInfo.h>
#include <uavAP/MissionControl/GlobalPlanner/ApproachPlanner/ApproachPlanner.h>
#include <uavAP/MissionControl/GlobalPlanner/ApproachPlanner/ApproachPlannerParams.h>

TEST_CASE("Approach Planner Test 1")
{
    Waypoint wp0; // current location
    wp0.location = Vector3(0,0,10);
    wp0.direction = Vector3(1,0,0);

    Waypoint wp1; // current location
    wp1.location = Vector3(100,100,40);
    wp1.direction = Vector3(0,1,0);

    ApproachPlanner ap;

    // can check trajectory here?
    trajectory = ap.createCatmulRomSplines(wp0, wp1);

    // can check mission here? setMission doesn't return anything
    ap.setMission(wp0, wp1);
}