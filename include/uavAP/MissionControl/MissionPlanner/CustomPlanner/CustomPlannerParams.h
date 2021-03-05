//
// Created by mirco on 26.06.20.
//

#ifndef UAVAP_CUSTOMPLANNERPARAMS_H
#define UAVAP_CUSTOMPLANNERPARAMS_H

#include <cpsCore/Utilities/LinearAlgebra.h>
#include <uavAP/MissionControl/MissionPlanner/Mission.h>
#include <unordered_map>

struct CustomPlannerParams
{
	Parameter<FloatingType> defaultVelocity = {{}, "default_velocity", true};
	Parameter<FloatingType> defaultAltitude = {{}, "default_altitude", true};
	Parameter<std::string> defaultMission = {"default_mission", "default_mission", false};
	Parameter<bool> useApproach = {false, "use_approach", false};

	Parameter<std::map<std::string, Mission>> missions = {{}, "missions", true};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & defaultVelocity;
		c & defaultAltitude;
		c & defaultMission;
		c & useApproach;
		c & missions;
	}
};


#endif //UAVAP_CUSTOMPLANNERPARAMS_H
