//
// Created by mirco on 24.06.20.
//

#ifndef UAVAP_LOGPARSERPARAMS_H
#define UAVAP_LOGPARSERPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>

struct LogParserParams
{
	Parameter<std::string> logFilePath = {"", "log_file_path", true};
	Parameter<std::string> logHeaderPath = {"", "log_header_path", true};
	Parameter<std::string> alvoloConfigPath = {"/usr/local/config/alvolo.json", "alvolo_config_path", true};
	Parameter<unsigned> periodUs = {10000, "period_us", true};
	Parameter<unsigned> offsetSecs = {0, "offset_secs", false};
	Parameter<bool> internalImu = {false, "internal_imu", true};
	Parameter<bool> externalGps = {false, "external_gps", true};
	Parameter<bool> useEuler = {true, "use_euler", true};
	Parameter<bool> useAirspeed = {true, "use_airspeed", true};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & logFilePath;
		c & logHeaderPath;
		c & alvoloConfigPath;
		c & periodUs;
		c & offsetSecs;
		c & internalImu;
		c & externalGps;
		c & useEuler;
		c & useAirspeed;
	}
};

#endif //UAVAP_LOGPARSERPARAMS_H
