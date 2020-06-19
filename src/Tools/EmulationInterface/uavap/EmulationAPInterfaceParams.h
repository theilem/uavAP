//
// Created by mirco on 16.06.20.
//

#ifndef UAVAP_EMULATIONAPINTERFACEPARAMS_H
#define UAVAP_EMULATIONAPINTERFACEPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>

class EmulationAPInterfaceParams
{

	Parameter<std::string> serialPort = {"/dev/tty0", "serial_port", true};

	template<typename Config>
	void
	configure(Config& c)
	{
		c & serialPort;
	}


};


#endif //UAVAP_EMULATIONAPINTERFACEPARAMS_H
