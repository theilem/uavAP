//
// Created by mirco on 03.07.20.
//

#ifndef UAVAP_EMULATIONDIRECTINTERFACEPARAMS_H
#define UAVAP_EMULATIONDIRECTINTERFACEPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>

struct EmulationDirectInterfaceParams
{
	Parameter<std::string> sensorDataTarget = {"sensor_data", "sensor_data_target", false};
	Parameter<std::string> controllerOutputTarget = {"controller_output", "controller_output_target", false};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & sensorDataTarget;
		c & controllerOutputTarget;
	}
};

#endif //UAVAP_EMULATIONDIRECTINTERFACEPARAMS_H
