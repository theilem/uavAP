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
/**
 *  @file         PIDControllerTest_Airplane.cpp
 *  @author  Mirco Theile
 *  @date      26 June 2017
 *  @brief      UAV Autopilot Airplane PID Controller Test Source File
 *
 *  Description
 */

#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "uavAP/Core/Runner/AggregatableRunner.h"
#include <boost/test/unit_test.hpp>

#include "uavAP/FlightControl/Controller/ControlElements/EvaluableControlElements.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDController.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/Scheduler/MicroSimulator.h"

/* Experimental Channel Mixing*/
class ChannelMixingTest: public IChannelMixing, public IAggregatableObject
{
public:
	void
	setControllerOutput(const ControllerOutput& control) override
	{
		controlLog = control;
	}

	void
	notifyAggregationOnUpdate(const Aggregator& agg)
	{
	}

	ControllerOutput controlLog;
};

/* Initialize Test Suite*/
BOOST_AUTO_TEST_SUITE(PIDControllerTest_Airplane)

/* Perform Tests for P Control */
BOOST_AUTO_TEST_CASE(PControl)
{
	/* Definitions */
	auto PIDControllerTest = std::make_shared<PIDController>();
	auto channelMixingTest = std::make_shared<ChannelMixingTest>();
	auto scheduler = std::make_shared<MicroSimulator>();
	auto data = std::make_shared<SensingActuationIO>();
	auto runner = std::make_shared<AggregatableRunner>();
	Configuration configuration;
	ControllerTarget target;

	/* Load General Configurations */
	boost::property_tree::read_json("FlightControl/Controller/config/P_Airplane.json",
			configuration);

	/* Initialize Test Controller */
	Aggregator::aggregate(
	{ PIDControllerTest, channelMixingTest, scheduler, data, runner });
	BOOST_REQUIRE(PIDControllerTest->configure(configuration));
	BOOST_REQUIRE(!runner->runAllStages());

	/* Set Test Case 1 Targets */
	target.velocity[0] = 1;
	target.velocity[2] = 2;
	target.yawRate = 3;

	/* Store Test Case 1 Targets to Test Controller */
	PIDControllerTest->setControllerTarget(target);

	/* Perform Test Case 1 */
	BOOST_CHECK_EQUAL(scheduler->simulate(Milliseconds(5)), 1);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.rollOutput, (M_PI / 6), 0.01);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.pitchOutput, (M_PI / 6), 0.01);
	BOOST_CHECK_EQUAL(channelMixingTest->controlLog.yawOutput, 0);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.throttleOutput, 0, 0.01);

	/* Set Test Case 2 Targets */
	target.velocity[0] = 2;
	target.velocity[2] = -0.5;
	target.yawRate = 0;

	/* Store Test Case 2 Targets to Test Controller */
	PIDControllerTest->setControllerTarget(target);

	/* Perform Test Case 2 */
	BOOST_CHECK_EQUAL(scheduler->simulate(Milliseconds(10)), 1);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.rollOutput, 0, 0.01);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.pitchOutput, -0.5, 0.01);
	BOOST_CHECK_EQUAL(channelMixingTest->controlLog.yawOutput, 0);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.throttleOutput, 1, 0.01);
}

/* Perform Tests for PD Control */
BOOST_AUTO_TEST_CASE(PDControl)
{
	/* Definitions */
	auto PIDControllerTest = std::make_shared<PIDController>();
	auto channelMixingTest = std::make_shared<ChannelMixingTest>();
	auto scheduler = std::make_shared<MicroSimulator>();
	auto data = std::make_shared<SensingActuationIO>();
	auto runner = std::make_shared<AggregatableRunner>();
	Configuration configuration;
	ControllerTarget target;

	/* Load General Configurations */
	boost::property_tree::read_json("FlightControl/Controller/config/PD_Airplane.json",
			configuration);

	/* Initialize Test Controller */
	Aggregator::aggregate(
	{ PIDControllerTest, channelMixingTest, scheduler, data, runner });
	BOOST_REQUIRE(PIDControllerTest->configure(configuration));
	BOOST_REQUIRE(!runner->runAllStages());

	/* Set Test Case 1 Data*/
	target.velocity[0] = 1;
	target.velocity[2] = 0.5;
	target.yawRate = 0.5;
	data->sensorData.acceleration = Vector3(1, 0, 0.5);
	data->sensorData.angularAcc = Vector3(0, 0, 0.5);

	/* Store Test Case 1 Targets to Test Controller */
	PIDControllerTest->setControllerTarget(target);

	/* Perform Test Case 1 */
	BOOST_CHECK_EQUAL(scheduler->simulate(Milliseconds(5)), 1);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.rollOutput, 0, 0.01);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.pitchOutput, 0, 0.01);
	BOOST_CHECK_EQUAL(channelMixingTest->controlLog.yawOutput, 0);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.throttleOutput, -1, 0.01);

	/* Set Test Case 2 Data */
	target.velocity[0] = 1;
	target.velocity[2] = 0.5;
	target.yawRate = 0.5;
	data->sensorData.acceleration = Vector3(0.5, 0, 0.2);
	data->sensorData.angularAcc = Vector3(0, 0, 0.2);

	/* Store Test Case 2 Targets to Test Controller */
	PIDControllerTest->setControllerTarget(target);

	/* Perform Test Case 2 */
	BOOST_CHECK_EQUAL(scheduler->simulate(Milliseconds(10)), 1);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.rollOutput, 0.3, 0.01);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.pitchOutput, 0.3, 0.01);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.throttleOutput, -0.5, 0.01);

	/* Set Test Case 3 Data */
	target.velocity[0] = 1;
	target.velocity[2] = 0.5;
	target.yawRate = 0.5;
	data->sensorData.acceleration = Vector3(0.5, 0, 1.0);
	data->sensorData.angularAcc = Vector3(0, 0, 1.0);

	/* Store Test Case 3 Targets to Test Controller */
	PIDControllerTest->setControllerTarget(target);

	/* Perform Test Case 3 */
	BOOST_CHECK_EQUAL(scheduler->simulate(Milliseconds(10)), 1);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.rollOutput, -0.5, 0.01);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.pitchOutput, -0.5, 0.01);
	BOOST_CHECK_EQUAL(channelMixingTest->controlLog.yawOutput, 0);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.throttleOutput, -0.5, 0.01);
}

/* Perform Tests for PFF Control */
BOOST_AUTO_TEST_CASE(PFFControl)
{
	/* Definitions */
	auto PIDControllerTest = std::make_shared<PIDController>();
	auto channelMixingTest = std::make_shared<ChannelMixingTest>();
	auto scheduler = std::make_shared<MicroSimulator>();
	auto data = std::make_shared<SensingActuationIO>();
	auto runner = std::make_shared<AggregatableRunner>();
	Configuration configuration;
	ControllerTarget target;

	/* Load General Configurations */
	boost::property_tree::read_json("FlightControl/Controller/config/PFF_Airplane.json",
			configuration);

	/* Initialize Test Controller */
	Aggregator::aggregate(
	{ PIDControllerTest, channelMixingTest, scheduler, data, runner });
	BOOST_REQUIRE(PIDControllerTest->configure(configuration));
	BOOST_REQUIRE(!runner->runAllStages());

	/* Set Test Case 1 Data */
	target.velocity[0] = 1.0;
	target.velocity[2] = 0.5;
	target.yawRate = 0.5;
	data->sensorData.velocity = Vector3(0, 0, 0.5);
	data->sensorData.groundSpeed = 1.0;
	data->sensorData.attitude = Vector3(0.5, 0.5, 0);
	data->sensorData.angularRate = Vector3(0, 0, 0.5);

	/* Store Test Case 1 Targets to Test Controller */
	PIDControllerTest->setControllerTarget(target);

	/* Perform Test Case 1 */
	BOOST_CHECK_EQUAL(scheduler->simulate(Milliseconds(5)), 1);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.rollOutput, 0.5, 0.01);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.pitchOutput, 0.5, 0.01);
	BOOST_CHECK_EQUAL(channelMixingTest->controlLog.yawOutput, 0);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.throttleOutput, 0, 0.01);
}

/* Perform Tests for PI Control */
BOOST_AUTO_TEST_CASE(PIControl)
{
	/* Definitions */
	auto PIDControllerTest = std::make_shared<PIDController>();
	auto channelMixingTest = std::make_shared<ChannelMixingTest>();
	auto scheduler = std::make_shared<MicroSimulator>();
	auto data = std::make_shared<SensingActuationIO>();
	auto runner = std::make_shared<AggregatableRunner>();
	Configuration configuration;
	ControllerTarget target;

	/* Load General Configurations */
	boost::property_tree::read_json("FlightControl/Controller/config/PI_Airplane.json",
			configuration);

	/* Initialize Test Controller */
	Aggregator::aggregate(
	{ PIDControllerTest, channelMixingTest, scheduler, data, runner });
	BOOST_REQUIRE(PIDControllerTest->configure(configuration));
	BOOST_REQUIRE(!runner->runAllStages());

	/* Set Test Case 1 Data */
	target.velocity[0] = 1.0;
	target.velocity[2] = 0.1;
	target.yawRate = 0.1;
	data->sensorData.timestamp = scheduler->now();

	/* Store Test Case 1 Targets to Test Controller */
	PIDControllerTest->setControllerTarget(target);

	/* Perform Test Case 1 */
	BOOST_CHECK_EQUAL(scheduler->simulate(Milliseconds(5)), 1);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.rollOutput, 0.1, 0.01);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.pitchOutput, 0.1, 0.01);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.throttleOutput, 0, 0.01);

	/* Set Test Case 2 Data */
	target.velocity[0] = 1.0;
	target.velocity[2] = 0.1;
	target.yawRate = 0.1;
	data->sensorData.timestamp = data->sensorData.timestamp + Seconds(1);

	/* Store Test Case 2 Targets to Test Controller */
	PIDControllerTest->setControllerTarget(target);

	/* Perform Test Case 2 */
	BOOST_CHECK_EQUAL(scheduler->simulate(Milliseconds(10)), 1);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.rollOutput, 0.4, 0.01);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.pitchOutput, 0.4, 0.01);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.throttleOutput, 1, 0.01);

	/* Set Test Case 3 Data */
	target.velocity[0] = 1.0;
	target.velocity[2] = 0.1;
	target.yawRate = 0.1;
	data->sensorData.timestamp = data->sensorData.timestamp + Seconds(1);
	data->sensorData.velocity = Vector3(0, 0, 0.1);
	data->sensorData.groundSpeed = 1.0;
	data->sensorData.angularRate = Vector3(0, 0, 0.1);

	/* Store Test Case 3 Targets to Test Controller */
	PIDControllerTest->setControllerTarget(target);

	/* Perform Test Case 3 */
	BOOST_CHECK_EQUAL(scheduler->simulate(Milliseconds(10)), 1);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.rollOutput, 0.4, 0.01);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.pitchOutput, 0.4, 0.01);
	BOOST_CHECK_CLOSE(channelMixingTest->controlLog.throttleOutput, 0, 0.01);
}

/* Perform Tests for PID Derivative */
BOOST_AUTO_TEST_CASE(PIDDerivative)
{
	double target;
	double sensor;
	double out;
	TimePoint timepoint(Date(2000, 1, 1), Milliseconds(0));
	sensor = 0;
	target = 1;
	Control::PID::Parameters p = Control::PID::Parameters();
	p.kp = 1;
	p.ki = 0;
	p.kd = 1;
	p.ff = 0;
	p.imax = 0;
	Control::ControlEnvironment controlEnv(&timepoint);
	auto inTarget = controlEnv.addInput(&target);
	auto inSensor = controlEnv.addInput(&sensor);
	auto PID = controlEnv.addPID(inTarget, inSensor, p);
	controlEnv.addOutput(PID, &out);
	controlEnv.evaluate();
	BOOST_CHECK_CLOSE(out, 1, 0.01);
	sensor = 0.5;
	timepoint = timepoint + Milliseconds(500);
	controlEnv.evaluate();
	BOOST_CHECK_CLOSE(out, -0.5, 0.01);
}

/* Terminate Test Suite*/
BOOST_AUTO_TEST_SUITE_END()
