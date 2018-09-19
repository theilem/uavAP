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
	std::cout << "Roll: 		" << control.rollOutput << std::endl;
	std::cout << "Pitch: 		" << control.pitchOutput << std::endl;
	std::cout << "Yaw: 		" << control.yawOutput << std::endl;
	std::cout << "Throttle: 	" << control.throttleOutput << std::endl;
}

void
dispSens(const SensorData& sd)
{
	if (!showSensor)
		return;
	std::cout << "Position: 	[" << sd.position.x() << "; 	" << sd.position.y() << "; 	"
			<< sd.position.z() << "]" << std::endl;
	std::cout << "Velocity: 	[" << sd.velocity.x() << "; 	" << sd.velocity.y() << "; 	"
			<< sd.velocity.z() << "]" << std::endl;
	std::cout << "Acceleration: 	[" << sd.acceleration.x() << "; 	" << sd.acceleration.y()
			<< "; 	" << sd.acceleration.z() << "]" << std::endl;
	std::cout << "Attitude: 	[" << sd.attitude.x() * 180 / M_PI << "; 	" << sd.attitude.y() * 180 / M_PI<< "; 	"
			<< sd.attitude.z() * 180 / M_PI << "]" << std::endl;
	std::cout << "AngularRate: 	[" << sd.angularRate.x() << "; 	" << sd.angularRate.y() << "; 	"
			<< sd.angularRate.z() << "]" << std::endl;
	std::cout << "AngularAcc: 	[" << sd.angularAcc.x() << "; 	" << sd.angularAcc.y() << "; 	"
			<< sd.angularAcc.z() << "]" << std::endl;
	std::cout << "Airspeed: 	" << sd.airSpeed << std::endl;
	std::cout << "Groundspeed: 	" << sd.groundSpeed << std::endl << std::endl;

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

	SimpleRunner runner(agg);
	runner.runAllStages();
	ControllerOutput data;

	std::string input;

	while (1)
	{
		input.clear();
		std::cout << "Enter shared memory object to listen to: " << std::endl;
		std::cout << "SensorData: sensor" << std::endl;
		std::cout << "ControllerOutput: control" << std::endl;
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
