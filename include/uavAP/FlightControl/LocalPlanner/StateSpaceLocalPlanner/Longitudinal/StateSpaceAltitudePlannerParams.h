//
// Created by seedship on 4/19/21.
//

#ifndef UAVAP_STATESPACEALTITUDEPLANNERPARAMS_H
#define UAVAP_STATESPACEALTITUDEPLANNERPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>

struct StateSpaceAltitudePlannerParams
{
	Parameter<VectorN<7>> k = {{}, "k", true};
	Parameter<FloatingType> safetyVelocity = {55.0, "safety_velocity", true};
	Parameter<FloatingType> safetyAltitude = {2500.0, "safety_altitude", true};
	Parameter<Angle<FloatingType>> maxPitchTarget = {Angle<FloatingType>(15), "max_pitch_target", true};
	Parameter<Angle<FloatingType>> minPitchTarget = {Angle<FloatingType>(-15), "min_pitch_target", true};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & k;
		c & safetyVelocity;
		c & safetyAltitude;
		c & maxPitchTarget;
		c & minPitchTarget;
	}
};


#endif //UAVAP_STATESPACEPLANNERALTITUDEPARAMS_H
