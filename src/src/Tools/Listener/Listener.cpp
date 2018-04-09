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
#include "uavAP/Core/Runner/SimpleRunner.h"
#include "uavAP/FlightControl/FlightControlHelper.h"

#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/API/ap_ext/ApExtManager.h"

bool showControl = false;
bool showSensor = false;
bool showOutput = false;

void
dispControl(const ControllerOutput& control)
{
    if (!showControl)
        return;
    std::cout << "Roll: \t\t" << control.rollOutput << std::endl;
    std::cout << "Pitch: \t\t" << control.pitchOutput << std::endl;
    std::cout << "Yaw: \t\t" << control.yawOutput << std::endl;
    std::cout << "Throttle: \t" << control.throttleOutput << std::endl;
    std::cout << "Collective: \t" << control.collectiveOutput << std::endl << std::endl;
}

void
dispOutput(const ApExtManager::OutPWM& control)
{
    if (!showOutput)
        return;
    for (int i = 0; i < 7; ++i)
    {
        std::cout << "Channel " << i << ": " << control.ch[i] << std::endl;
    }
}

void
dispSens(const SensorData& sd)
{
    if (!showSensor)
        return;
    std::cout << "Velocity: \t[" << sd.velocity.x() << "; \t" << sd.velocity.y() << "; \t"
              << sd.velocity.z() << "]" << std::endl;
    std::cout << "Acceleration: \t[" << sd.acceleration.x() << "; \t" << sd.acceleration.y()
              << "; \t" << sd.acceleration.z() << "]" << std::endl;
    std::cout << "Attitude: \t[" << sd.attitude.x() << "; \t" << sd.attitude.y() << "; \t"
              << sd.attitude.z() << "]" << std::endl;
    std::cout << "AngularRate: \t[" << sd.angularRate.x() << "; \t" << sd.angularRate.y() << "; \t"
              << sd.angularRate.z() << "]" << std::endl;
    std::cout << "AngularAcc: \t[" << sd.angularAcc.x() << "; \t" << sd.angularAcc.y() << "; \t"
              << sd.angularAcc.z() << "]" << std::endl;
    std::cout << "Airspeed: \t" << sd.velocityAir << std::endl;
    std::cout << "Groundspeed: \t" << sd.velocityGround << std::endl << std::endl;

}

int
main(int argc, char** argv)
{
    APLogger::instance()->setLogLevel(LogLevel::DEBUG);
    auto ipc = std::make_shared<IPC>();
    auto tp = std::make_shared<SystemTimeProvider>();
    auto sched = std::make_shared<MultiThreadingScheduler>();
    auto agg = Aggregator::aggregate(
    { ipc, tp, sched });

    ipc->subscribeOnSharedMemory<ControllerOutput>("actuation", &dispControl);
    ipc->subscribeOnSharedMemory<SensorData>("sensor_data", &dispSens);
    ipc->subscribeOnSharedMemory<ApExtManager::OutPWM>("output_pwm", &dispOutput);

    SimpleRunner runner(agg);
    runner.runAllStages();
    ControllerOutput data;

    std::string input;

    while (1)
    {
        input.clear();
        std::cout << "\nEnter shared memory object to listen to: " << std::endl;
        std::cout << "SensorData: sensor" << std::endl;
        std::cout << "ControllerOutput: control" << std::endl;
        std::cout << "APExternal output: output" << std::endl << std::endl;
        std::cout << "Pause: p, Quit: q" << std::endl;
        std::cin >> input;
        if (input.compare("q") == 0)
        {
            break;
        }
        else if (input.compare("p") == 0)
        {
            showSensor = false;
            showControl = false;
            showOutput = false;
        }
        else if (input.compare("sensor") == 0)
        {
            showSensor = true;
            showControl = false;
            showOutput = false;
        }
        else if (input.compare("control") == 0)
        {
            showSensor = false;
            showControl = true;
            showOutput = false;
        }
        else if (input.compare("output") == 0)
        {
            showSensor = false;
            showControl = false;
            showOutput = true;
        }

    }

    return 0;
}

