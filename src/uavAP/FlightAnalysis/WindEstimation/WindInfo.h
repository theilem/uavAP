/*
 * WindInfo.h
 *
 *  Created on: Feb 8, 2019
 *      Author: mirco
 */

#ifndef UAVAP_FLIGHTANALYSIS_WINDESTIMATION_WINDINFO_H_
#define UAVAP_FLIGHTANALYSIS_WINDESTIMATION_WINDINFO_H_
#include <uavAP/Core/LinearAlgebra.h>

struct WindInfo
{
	FloatingType speed;
	Vector3 direction;
};

namespace dp
{
template<class Archive, typename T>
inline void
serialize(Archive& ar, WindInfo& t)
{
	ar & t.speed;
	ar & t.direction;
}
}

#endif /* UAVAP_FLIGHTANALYSIS_WINDESTIMATION_WINDINFO_H_ */
