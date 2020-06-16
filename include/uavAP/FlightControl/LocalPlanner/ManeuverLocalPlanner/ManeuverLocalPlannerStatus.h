/*
 * ManeuverLocalPlannerStatus.h
 *
 *  Created on: Jun 28, 2019
 *      Author: mirco
 */

#ifndef UAVAP_FLIGHTCONTROL_LOCALPLANNER_MANEUVERLOCALPLANNER_MANEUVERLOCALPLANNERSTATUS_H_
#define UAVAP_FLIGHTCONTROL_LOCALPLANNER_MANEUVERLOCALPLANNER_MANEUVERLOCALPLANNERSTATUS_H_
#include <cpsCore/Utilities/DataPresentation/detail/SerializeCustom.h>
#include <cpsCore/Utilities/LinearAlgebra.h>

struct ManeuverLocalPlannerStatus : public SerializeCustom
{
	FloatingType velocityTarget;
	FloatingType headingTarget;
	FloatingType yawRateTarget;
	FloatingType climbAngleTarget;
	uint8_t currentPathSection;
	bool isInApproach;
};

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, ManeuverLocalPlannerStatus& t)
{
	ar & t.velocityTarget;
	ar & t.headingTarget;
	ar & t.yawRateTarget;
	ar & t.climbAngleTarget;
	ar & t.currentPathSection;
	ar & t.isInApproach;
}
}

#endif /* UAVAP_FLIGHTCONTROL_LOCALPLANNER_MANEUVERLOCALPLANNER_MANEUVERLOCALPLANNERSTATUS_H_ */
