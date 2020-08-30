//
// Created by mirco on 27.08.20.
//

#ifndef UAVAP_APEXTMANAGERPARAMS_H
#define UAVAP_APEXTMANAGERPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>

struct ApExtManagerParams
{
	Parameter<bool> internalImu = {true, "internal_imu", true};
	Parameter<bool> externalGps = {false, "external_gps", true};
	Parameter<bool> useAirspeed = {true, "use_airspeed", true};
	Parameter<bool> useEuler = {true, "use_euler", true};
	Parameter<bool> courseAsHeading = {false, "course_as_heading", true};
//	Parameter<Optional<Eigen::Vector4d>> rotationOffset = {std::nullopt, "rotation_offset", false};
	Parameter<Duration> gpsTimeout = {Seconds(1), "gps_timeout_ms", false};
	Parameter<Duration> airspeedTimeout = {Milliseconds(500), "airspeed_timeout_ms", false};
	Parameter<std::map<std::string, unsigned>> channelNaming = {{}, "channel_naming", true};
	Parameter<Control::LowPassFilterParams> airspeedFilter = {{}, "airspeed_filter", true};
	Parameter<Configuration> channelMixing = {Configuration(), "channel_mixing", true};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & internalImu;
		c & externalGps;
		c & useAirspeed;
		c & useEuler;
		c & courseAsHeading;
//		c & rotationOffset;
		c & gpsTimeout;
		c & airspeedTimeout;
		c & channelNaming;
		c & airspeedFilter;
		c & channelMixing;
	}

};


#endif //UAVAP_APEXTMANAGERPARAMS_H
