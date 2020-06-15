/*
 * Rectanguloid.h
 *
 *  Created on: Jun 28, 2019
 *      Author: mirco
 */

#ifndef UAVAP_MISSIONCONTROL_GEOFENCING_RECTANGULOID_H_
#define UAVAP_MISSIONCONTROL_GEOFENCING_RECTANGULOID_H_

#include <cpsCore/Utilities/LinearAlgebra.h>

struct Rectanguloid
{
	Vector3 center;
	FloatingType majorSideLength;
	FloatingType minorSideLength;
	FloatingType majorSideOrientation;
	FloatingType height;
};

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, Rectanguloid& t)
{
	ar & t.center;
	ar & t.majorSideLength;
	ar & t.minorSideLength;
	ar & t.majorSideOrientation;
	ar & t.height;
}
}

#endif /* UAVAP_MISSIONCONTROL_GEOFENCING_RECTANGULOID_H_ */
