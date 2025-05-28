//
// Created by Mirco Theile on 27/5/25.
//

#ifndef APPROACH_H
#define APPROACH_H
#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"

using Pose = std::pair<Vector3, Vector3>; // Position and Direction

PathSections
approach(const Pose& start, const Pose& end, FloatingType velocity, FloatingType slope, FloatingType radius);

PathSections
approach(const Pose& start, std::shared_ptr<Orbit> end, FloatingType velocity, FloatingType slope, FloatingType radius);

std::shared_ptr<Orbit>
poseToOrbit(const Pose& pose, OrbitDirection direction, FloatingType radius, FloatingType vel);


#endif //APPROACH_H
