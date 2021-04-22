//
// Created by seedship on 4/19/21.
//

#ifndef UAVAP_STATESPACEALTITUDEPLANNERSTATUS_H
#define UAVAP_STATESPACEALTITUDEPLANNERSTATUS_H

struct StateSpaceAltitudePlannerStatus : public SerializeCustom
{
	FloatingType velocityTarget;
	FloatingType headingTarget;
	FloatingType yawRateTarget;
	FloatingType pitchTarget;
	uint8_t currentPathSection;
	bool isInApproach;
};

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, StateSpaceAltitudePlannerStatus& t)
{
	ar & t.velocityTarget;
	ar & t.headingTarget;
	ar & t.yawRateTarget;
	ar & t.pitchTarget;
	ar & t.currentPathSection;
	ar & t.isInApproach;
}
}

#endif //UAVAP_STATESPACEALTITUDEPLANNERSTATUS_H
