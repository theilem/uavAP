//
// Created by mirco on 19.02.21.
//

#ifndef UAVAP_MANEUVERANALYSISPARAMS_H
#define UAVAP_MANEUVERANALYSISPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>

struct ManeuverAnalysisParams
{
	Parameter<std::string> logPath = {"", "log_path", true};
};
#endif //UAVAP_MANEUVERANALYSISPARAMS_H
