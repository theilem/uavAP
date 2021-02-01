//
// Created by seedship on 1/30/21.
//

#ifndef UAVAP_PITCHSTATESPACECONTROLLERTARGET_H
#define UAVAP_PITCHSTATESPACECONTROLLERTARGET_H


#include <cpsCore/Utilities/DataPresentation/detail/SerializeCustom.h>

struct PitchStateSpaceControllerTarget : SerializeCustom
{
	FloatingType velocity;
	FloatingType pitch;

//	uint32_t sequenceNr; //Trace sequence number to get timing

	PitchStateSpaceControllerTarget() :
			velocity(0), pitch(0)
	{
	}
};

enum class PitchStateSpaceControllerTargets
{
	VELOCITY, PITCH
};

ENUMMAP_INIT(PitchStateSpaceControllerTargets, {
{ PitchStateSpaceControllerTargets::VELOCITY, "velocity" },
{ PitchStateSpaceControllerTargets::PITCH, "pitch" }
});

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, PitchStateSpaceControllerTarget& t)
{
	ar & t.velocity;
	ar & t.pitch;
}

}

#endif //UAVAP_PITCHSTATESPACECONTROLLERTARGET_H
