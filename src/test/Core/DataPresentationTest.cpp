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
 * DataPresentationTest.cpp
 *
 *  Created on: Jul 17, 2017
 *      Author: mircot
 */

#include <boost/test/unit_test.hpp>
#include "uavAP/Core/LinearAlgebra.h"
#include <iostream>
#include <boost/thread/thread_time.hpp>
#include "uavAP/Core/DataPresentation/APDataPresentation/APDataPresentation.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/BinaryToArchive.h"
#include "uavAP/Core/SensorData.h"

#include "uavAP/Core/Logging/APLogger.h"

BOOST_AUTO_TEST_SUITE(DataPresentationTest)



void
printString(std::string str)
{
	for (unsigned int i = 0; i < str.size(); i++)
	{
		std::cout << (unsigned int)((uint8_t)str[i]) << "|";
	}
	std::cout << std::endl;
}

BOOST_AUTO_TEST_CASE(sensor_data)
{
	SensorData test;

	test.position = Vector3(1,2,3);
	test.timestamp = boost::get_system_time();

	APDataPresentation dp;
	Packet packet = dp.serialize(test, Content::SENSOR_DATA);

	APLogger::instance()->setLogLevel(LogLevel::NONE);
	Packet packet2;
	BOOST_REQUIRE_NO_THROW(packet2 = dp.serialize(test, Content::LOCAL_PLANNER_STATUS));
	APLogger::instance()->setLogLevel(LogLevel::WARN);

	BOOST_CHECK_EQUAL(packet2.getSize(), (unsigned int)0);

	Content receivedContent;
	boost::any check = dp.deserialize(packet, receivedContent);

	BOOST_REQUIRE(receivedContent == Content::SENSOR_DATA);
	SensorData sensorCheck;
	BOOST_REQUIRE_NO_THROW(sensorCheck = boost::any_cast<SensorData>(check));

	BOOST_CHECK_EQUAL(test.position, sensorCheck.position);
	BOOST_CHECK_EQUAL(test.timestamp, sensorCheck.timestamp);
}

BOOST_AUTO_TEST_CASE(ap_data_presentation_test)
{
	double test1 = 5.312;
	double test2 = 1151351.21;

	std::string packet;
	BinaryToArchive toArchive(packet);

	toArchive << test1;
	toArchive << test2;

	std::string packet2;
	BinaryToArchive toArchive2(packet2);

	toArchive2 << test1;

	std::string packet3;
	BinaryToArchive toArchive3(packet3);

	toArchive3 << test2;

	BOOST_CHECK_EQUAL(packet.size(), (unsigned int)16);

	BinaryFromArchive fromArchive(packet);
	BinaryFromArchive fromArchive3(packet3);

	double check1 = 0;
	double check2 = 0;
	fromArchive >> check1;
	fromArchive >> check2;

	BOOST_CHECK_EQUAL(test1, check1);
	BOOST_CHECK_EQUAL(test2, check2);

	fromArchive3 >> check2;

	BOOST_CHECK_EQUAL(test2, check2);
}

BOOST_AUTO_TEST_CASE(ap_data_presentation_test_002)
{
	Vector3 test(1,2,3);
	ConstraintParams params;
	params.min = 7.21;
	params.max = 351.315331;

	std::string packet;
	BinaryToArchive toArchive(packet);

	toArchive << test << params;

	BOOST_CHECK_EQUAL(packet.size(), 5 * sizeof(double));

	BinaryFromArchive fromArchive(packet);

	Vector3 check(0,0,0);
	ConstraintParams checkParams;
	fromArchive >> check >> checkParams;

	BOOST_CHECK_EQUAL(test, check);
	BOOST_CHECK_EQUAL(params.min, checkParams.min);
	BOOST_CHECK_EQUAL(params.max, checkParams.max);

}


BOOST_AUTO_TEST_CASE(ap_data_presentation_test_003_maneuver)
{
	Vector3 test(1,2,3);
	ManeuverOverride ov;
	ov.overrideManeuverPlanner = true;
	ov.activation.activate = true;
	ov.activation.overridePitch = false;
	ov.activation.overrideRoll = true;
	ov.activation.overrideVelocity = false;
	ov.target.pitchTarget = 17.536;
	ov.target.rollTarget= 1.23;
	ov.target.velocityTarget = 20.5;

	std::string packet;
	BinaryToArchive toArchive(packet);

	toArchive << ov;

//	BOOST_CHECK_EQUAL(packet.size(), 5 * sizeof(double));

	BinaryFromArchive fromArchive(packet);

	ManeuverOverride checkOv;
	fromArchive >> checkOv;

	BOOST_CHECK_EQUAL(ov.overrideManeuverPlanner, checkOv.overrideManeuverPlanner);
	BOOST_CHECK_EQUAL(ov.activation.activate, checkOv.activation.activate);
	BOOST_CHECK_EQUAL(ov.activation.overridePitch, checkOv.activation.overridePitch);
	BOOST_CHECK_EQUAL(ov.activation.overrideRoll, checkOv.activation.overrideRoll);
	BOOST_CHECK_EQUAL(ov.activation.overrideVelocity, checkOv.activation.overrideVelocity);
	BOOST_CHECK_EQUAL(ov.target.pitchTarget, checkOv.target.pitchTarget);
	BOOST_CHECK_EQUAL(ov.target.rollTarget, checkOv.target.rollTarget);
	BOOST_CHECK_EQUAL(ov.target.velocityTarget, checkOv.target.velocityTarget);

}


BOOST_AUTO_TEST_CASE(ap_data_presentation_test_004_string)
{
	std::string s("Hallo ... yeah");

	APDataPresentation dp;
	Packet packet = dp.serialize(s, Content::SELECT_MISSION);

	Content content;
	auto check = dp.deserialize(packet, content);
	auto s1 = boost::any_cast<std::string>(check);


	BOOST_CHECK_EQUAL(s1.compare(s), 0);

}
BOOST_AUTO_TEST_SUITE_END()

