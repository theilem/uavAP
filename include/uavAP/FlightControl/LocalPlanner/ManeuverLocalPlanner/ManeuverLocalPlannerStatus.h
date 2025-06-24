/*
 * ManeuverLocalPlannerStatus.h
 *
 *  Created on: Jun 28, 2019
 *      Author: mirco
 */

#ifndef UAVAP_FLIGHTCONTROL_LOCALPLANNER_MANEUVERLOCALPLANNER_MANEUVERLOCALPLANNERSTATUS_H_
#define UAVAP_FLIGHTCONTROL_LOCALPLANNER_MANEUVERLOCALPLANNER_MANEUVERLOCALPLANNERSTATUS_H_
#include <memory>
#include <cpsCore/Utilities/DataPresentation/detail/SerializeCustom.h>
#include <cpsCore/Utilities/LinearAlgebra.h>
#include "uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h"

struct ManeuverLocalPlannerStatus : SerializeCustom
{
	FloatingType velocityTarget;
	FloatingType headingTarget;
	FloatingType yawRateTarget;
	FloatingType climbAngleTarget;
	std::shared_ptr<IPathSection> currentPathSection;
};

namespace dp
{
template<class Archive, typename >
void
serialize(Archive& ar, ManeuverLocalPlannerStatus& t)
{
	ar & t.velocityTarget;
	ar & t.headingTarget;
	ar & t.yawRateTarget;
	ar & t.climbAngleTarget;
	ar & t.currentPathSection;
}
}

#endif /* UAVAP_FLIGHTCONTROL_LOCALPLANNER_MANEUVERLOCALPLANNER_MANEUVERLOCALPLANNERSTATUS_H_ */
