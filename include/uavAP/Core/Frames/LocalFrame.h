//
// Created by seedship on 1/22/21.
//

#ifndef UAVAP_LOCALFRAME_H
#define UAVAP_LOCALFRAME_H

#include <cpsCore/Utilities/LinearAlgebra.h>
#include <cpsCore/Utilities/DataPresentation/detail/Split.h>

class SensorData;

struct LocalFrame
{
	LocalFrame();

	LocalFrame(const Vector3& origin, FloatingType yaw);

	void toLocalFrame(SensorData &sd);

	void toGlobalFrame(SensorData &sd);

	Vector3 origin;
	FloatingType yaw;
};


namespace dp
{
template<class Archive, typename Type>
inline void
load(Archive& ar, LocalFrame& t)
{
	Vector3 origin;
	FloatingType yaw;
	ar & origin;
	ar & yaw;
	t = LocalFrame(origin, yaw);
}

template<class Archive, typename Type>
inline void
store(Archive& ar, LocalFrame& t)
{
	ar & t.origin;
	ar & t.yaw;
}

template<class Archive, typename Type>
inline void
serialize(Archive& ar, LocalFrame& t)
{
	split(ar, t);
}
}

#endif //UAVAP_LOCALFRAME_H
