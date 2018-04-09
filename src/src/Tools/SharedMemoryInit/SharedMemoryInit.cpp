////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
// 
// This file is part of uavAP.
// 
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/*
 * OutputListener.cpp
 *
 *  Created on: Jul 27, 2017
 *      Author: mircot
 */

#include <boost/property_tree/json_parser.hpp>
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/Runner/SimpleRunner.h"
#include "uavAP/Core/SensorData.h"

#include "uavAP/Core/Logging/APLogger.h"



int
main(int argc, char** argv)
{
    APLogger::instance()->setLogLevel(LogLevel::DEBUG);
    auto ipc = std::make_shared<IPC>();


    SensorData sd;
    sd.timestamp = boost::get_system_time();
    sd.hasGPSFix = true;
    auto pub = ipc->publishOnSharedMemory<SensorData>("sensor_data");
    pub.publish(sd);

    std::string input;
    while (1)
    {
        input.clear();
        std::cin >> input;
        if (input.compare("q") == 0)
        {
            break;
        }
        if (input.compare("u") == 0)
        {
            sd.timestamp = boost::get_system_time();
            pub.publish(sd);
        }
    }

    return 0;
}

