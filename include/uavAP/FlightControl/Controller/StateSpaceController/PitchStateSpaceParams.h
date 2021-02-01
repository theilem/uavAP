//
// Created by seedship on 1/31/21.
//

#ifndef UAVAP_PITCHSTATESPACEPARAMS_H
#define UAVAP_PITCHSTATESPACEPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>

struct PitchStateSpaceParams
{
	Parameter<std::vector<FloatingType>> a = {{}, "a", true};
	Parameter<std::vector<FloatingType>> b = {{}, "b", true};
	Parameter<std::vector<FloatingType>> k = {{}, "k", true};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & a;
		c & b;
		c & k;
	}
};

#endif //UAVAP_PITCHSTATESPACEPARAMS_H
