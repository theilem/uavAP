/*
 * OutputListener.cpp
 *
 *  Created on: Jul 27, 2017
 *      Author: mircot
 */

#include <boost/property_tree/json_parser.hpp>

#include <cpsCore/Synchronization/SimpleRunner.h>
#include <cpsCore/Utilities/TimeProvider/SystemTimeProvider.h>
#include <cpsCore/Utilities/Scheduler/MultiThreadingScheduler.h>
#include <cpsCore/Utilities/SignalHandler/SignalHandler.h>

#include "uavAP/API/ap_ext/ApExtManager.h"
#include "uavAP/Core/SensorData.h"

#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <cpsCore/Utilities/IPC/IPC.h>

#include "uavAP/Core/Orientation/NED.h"
#include "uavAP/Core/Orientation/ConversionUtils.h"

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
	SensorData tmp = sd;
	NED::convert(tmp);

	directionalConversion(tmp.velocity, tmp.attitude, Frame::BODY, Orientation::NED);

	if (!showSensor)
		return;
	std::cout << "Position: 	[" << tmp.position.x() << "; 	" << tmp.position.y() << "; 	"
			<< tmp.position.z() << "]" << std::endl;
	std::cout << "Velocity: 	[" << tmp.velocity.x() << "; 	" << tmp.velocity.y() << "; 	"
			<< tmp.velocity.z() << "]" << std::endl;
	std::cout << "Acceleration: 	[" << tmp.acceleration.x() << "; 	" << tmp.acceleration.y()
			<< "; 	" << tmp.acceleration.z() << "]" << std::endl;
	std::cout << "Attitude: 	[" << radToDeg(tmp.attitude.x()) << "; 	" << radToDeg(tmp.attitude.y()) << "; 	"
			<< radToDeg(tmp.attitude.z()) << "]" << std::endl;
	std::cout << "AngularRate: 	[" << radToDeg(tmp.angularRate.x()) << "; 	" << radToDeg(tmp.angularRate.y()) << "; 	"
			<< radToDeg(tmp.angularRate.z()) << "]" << std::endl;
	std::cout << "α, β, χ:	[" << radToDeg(tmp.angleOfAttack) << "; 	" << radToDeg(tmp.angleOfSideslip) << "; 	"
								 << radToDeg(tmp.courseAngle) << "]" << std::endl;
	std::cout << "Airspeed: 	" << tmp.airSpeed << std::endl;
	std::cout << "Groundspeed: 	" << tmp.groundSpeed << std::endl << std::endl;
	std::cout << "Pressure: 	" << tmp.pressure << std::endl << std::endl;
	std::cout << "Temperature: 	" << tmp.temperature << std::endl << std::endl;

}

int
main(int argc, char** argv)
{
	CPSLogger::instance()->setLogLevel(LogLevel::DEBUG);
	auto ipc = std::make_shared<IPC>();
	auto dp = std::make_shared<DataPresentation>();
	auto tp = std::make_shared<SystemTimeProvider>();
	auto sched = std::make_shared<MultiThreadingScheduler>();
	auto sh = std::make_shared<SignalHandler>();
	auto agg = Aggregator::aggregate(
	{ ipc, dp, tp, sched, sh });

	ArchiveOptions opts;
	opts.compressDouble = true;
	dp->setParams(opts);

	ipc->subscribe<ControllerOutput>("actuation", &dispControl);
	ipc->subscribe<SensorData>("sensor_data", &dispSens);

	bool running = true;
	sh->subscribeOnExit([&running](){running = false;});

	SimpleRunner runner(agg);
	runner.runAllStages();
	ControllerOutput data;

	std::string input;

	while (running)
	{
		input.clear();
		std::cout << "Enter shared memory object to listen to: " << std::endl;
		std::cout << "SensorData: sensor" << std::endl;
		std::cout << "ControllerOutput: control" << std::endl;
		std::cout << "Pause: p, Quit: q" << std::endl;
		std::cin >> input;
		if (input == "q")
		{
			break;
		}
		else if (input == "p")
		{
			showSensor = false;
			showControl = false;
			showOutput = false;
		}
		else if (input == "sensor")
		{
			showSensor = true;
			showControl = false;
			showOutput = false;
		}
		else if (input == "control")
		{
			showSensor = false;
			showControl = true;
			showOutput = false;
		}
		else if (input == "output")
		{
			showSensor = false;
			showControl = false;
			showOutput = true;
		}

	}

	return 0;
}
