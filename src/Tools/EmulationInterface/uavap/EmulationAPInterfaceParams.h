//
// Created by mirco on 16.06.20.
//

#ifndef UAVAP_EMULATIONAPINTERFACEPARAMS_H
#define UAVAP_EMULATIONAPINTERFACEPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>

class EmulationAPInterfaceParams
{

	Parameter<std::string> serialPort = {"serial_port", "/dev/tty0", true};


};


#endif //UAVAP_EMULATIONAPINTERFACEPARAMS_H
