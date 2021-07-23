//
// Created by seedship on 1/31/21.
//

#ifndef UAVAP_PITCHSTATESPACEPARAMS_H
#define UAVAP_PITCHSTATESPACEPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>

struct PitchStateSpaceParams
{
	Parameter<Eigen::Matrix<FloatingType, 2, 6, Eigen::DontAlign>> k = {{}, "k", true};
	Parameter<FloatingType> tE = {0, "elevator_trim", true};
	Parameter<FloatingType> tT = {0, "throttle_trim", true};
	Parameter<Angle<FloatingType>> rP = {Angle<FloatingType>(0), "reference_pitch", true};
	Parameter<FloatingType> rU = {0, "reference_U", true};
	Parameter<FloatingType> rW = {0, "reference_W", true};
	Parameter<bool> inputClimbAngle = {false, "input_is_climb_angle", true};
	Parameter<Configuration> cascade = {{}, "cascade", true};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & k;
		c & tE;
		c & tT;
		c & rP;
		c & rU;
		c & rW;
		c & inputClimbAngle;
		c & cascade;
	}
};

#endif //UAVAP_PITCHSTATESPACEPARAMS_H
