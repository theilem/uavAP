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

#include "uavAP/Core/LinearAlgebra.h"
#include <iostream>
#include <boost/thread/thread_time.hpp>
#include "uavAP/Core/SensorData.h"
#include <boost/test/unit_test.hpp>
#include <uavAP/Core/DataPresentation/Content.h>

#include "uavAP/Core/Logging/APLogger.h"

#include <sstream>
#include <fstream>
#include <uavAP/FlightControl/LocalPlanner/LinearLocalPlanner/LinearLocalPlannerStatus.h>

#include "uavAP/Core/DataPresentation/BinarySerialization.hpp"
#include <uavAP/Core/DataPresentation/DataPresentation.h>

BOOST_AUTO_TEST_SUITE(DataPresentationTest)

void
printString(std::string str)
{
	for (unsigned int i = 0; i < str.size(); i++)
	{
		std::cout << (unsigned int) ((uint8_t) str[i]) << "|";
	}
	std::cout << std::endl;
}

BOOST_AUTO_TEST_CASE(sensor_data)
{
	SensorData test;

	test.position = Vector3(1, 2, 3);
	test.timestamp = Clock::now();

	DataPresentation dp;
	Packet packet = dp.serialize(test);
	dp.addHeader(packet, Content::SENSOR_DATA);

	Content receivedContent = dp.extractHeader<Content>(packet);

	BOOST_REQUIRE(receivedContent == Content::SENSOR_DATA);
	SensorData sensorCheck;
	BOOST_REQUIRE_NO_THROW(sensorCheck = dp.deserialize<SensorData>(packet));

	BOOST_CHECK_EQUAL(test.position, sensorCheck.position);
	BOOST_CHECK_EQUAL(test.timestamp.time_since_epoch().count(), sensorCheck.timestamp.time_since_epoch().count());
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

	BOOST_CHECK_EQUAL(packet.size(), (unsigned int )16);

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

BOOST_AUTO_TEST_CASE(ap_data_presentation_test_004_string)
{
	std::string s("Hallo ... yeah");

	DataPresentation dp;
	Packet packet = dp.serialize(s);
	dp.addHeader(packet, Content::SELECT_MISSION);

	Content content = dp.extractHeader<Content>(packet);
	auto check = dp.deserialize<std::string>(packet);

	BOOST_CHECK_EQUAL(s.compare(check), 0);

}

BOOST_AUTO_TEST_CASE(ap_data_presentation_test_005_compress_double)
{
	SensorData sd;
	sd.position = Vector3(M_PI, 2 * M_PI, 0.5 * M_PI);
	DataPresentation dp;
	Packet packet1 = dp.serialize(sd);
	dp.addHeader(packet1, Content::SENSOR_DATA);

	Content content = dp.extractHeader<Content>(packet1);
	BOOST_REQUIRE_EQUAL(static_cast<int>(content), static_cast<int>(Content::SENSOR_DATA));

	SensorData sdDes = dp.deserialize<SensorData>(packet1);
	BOOST_CHECK_EQUAL(sdDes.position, sd.position);

	DataPresentation dpCompress;
	Configuration config;
	config.add("compress_double", true);
	dpCompress.configure(config);
	Packet packet2 = dpCompress.serialize(sd);
	dpCompress.addHeader(packet2, Content::SENSOR_DATA);

	content = dpCompress.extractHeader<Content>(packet2);
	BOOST_REQUIRE_EQUAL(static_cast<int>(content), static_cast<int>(Content::SENSOR_DATA));

	sdDes = dpCompress.deserialize<SensorData>(packet2);

	BOOST_CHECK_CLOSE(sdDes.position.x(), sd.position.x(), 1e-4);
	BOOST_CHECK_CLOSE(sdDes.position.y(), sd.position.y(), 1e-4);
	BOOST_CHECK_CLOSE(sdDes.position.z(), sd.position.z(), 1e-4);
}

BOOST_AUTO_TEST_CASE(ap_data_presentation_test_006_file_archive)
{
	SensorData sd;
	sd.position = Vector3(M_PI, 2 * M_PI, 0.5 * M_PI);
	SensorData sd2;
	sd2.position = Vector3(3, 2, 0.5);

	std::ofstream fileOut;
	fileOut.open("test", std::ofstream::out | std::ofstream::binary);

	FileToArchive toArchive(fileOut);
	toArchive << sd << sd2 << sd << sd2;
	fileOut.close();

	SensorData sdRead;

	std::ifstream fileIn;
	fileIn.open("test", std::ifstream::in | std::ifstream::binary);

	FileFromArchive fromArchive(fileIn);
	fromArchive >> sdRead;
	BOOST_CHECK_EQUAL(sdRead.position, sd.position);
	fromArchive >> sdRead;
	BOOST_CHECK_EQUAL(sdRead.position, sd2.position);
	fromArchive >> sdRead;
	BOOST_CHECK_EQUAL(sdRead.position, sd.position);
	fromArchive >> sdRead;
	BOOST_CHECK_EQUAL(sdRead.position, sd2.position);

	fileIn.close();

	//Compressed double
	fileOut.open("test_compressed", std::ofstream::out | std::ofstream::binary);

	ArchiveOptions opt;
	opt.compressDouble.setValue(true);
	FileToArchive toArchive2(fileOut, opt);
	toArchive2 << sd << sd2 << sd << sd2;
	fileOut.close();

	fileIn.open("test_compressed", std::ifstream::in | std::ifstream::binary);

	FileFromArchive fromArchive2(fileIn, opt);
	fromArchive2 >> sdRead;
	BOOST_CHECK_CLOSE(sdRead.position.x(), sd.position.x(), 1e-4);
	BOOST_CHECK_CLOSE(sdRead.position.y(), sd.position.y(), 1e-4);
	BOOST_CHECK_CLOSE(sdRead.position.z(), sd.position.z(), 1e-4);
	fromArchive2 >> sdRead;
	BOOST_CHECK_CLOSE(sdRead.position.x(), sd2.position.x(), 1e-4);
	BOOST_CHECK_CLOSE(sdRead.position.y(), sd2.position.y(), 1e-4);
	BOOST_CHECK_CLOSE(sdRead.position.z(), sd2.position.z(), 1e-4);
	fromArchive2 >> sdRead;
	BOOST_CHECK_CLOSE(sdRead.position.x(), sd.position.x(), 1e-4);
	BOOST_CHECK_CLOSE(sdRead.position.y(), sd.position.y(), 1e-4);
	BOOST_CHECK_CLOSE(sdRead.position.z(), sd.position.z(), 1e-4);
	fromArchive2 >> sdRead;
	BOOST_CHECK_CLOSE(sdRead.position.x(), sd2.position.x(), 1e-4);
	BOOST_CHECK_CLOSE(sdRead.position.y(), sd2.position.y(), 1e-4);
	BOOST_CHECK_CLOSE(sdRead.position.z(), sd2.position.z(), 1e-4);

	fileIn.close();
}

BOOST_AUTO_TEST_CASE(binary_serialization_001_archive_options)
{
	SensorData sd;
	sd.position = Vector3(M_PI, 2 * M_PI, 0.5 * M_PI);

	Packet packet = dp::serialize(sd);
	auto sdDes = dp::deserialize<SensorData>(packet);
	BOOST_CHECK_EQUAL(sdDes.position, sd.position);

	ArchiveOptions opt;
	opt.compressDouble.setValue(true);
	packet = dp::serialize(sd,opt);
	sdDes = dp::deserialize<SensorData>(packet, opt);
	BOOST_CHECK_CLOSE(sdDes.position.x(), sd.position.x(), 1e-4);
	BOOST_CHECK_CLOSE(sdDes.position.y(), sd.position.y(), 1e-4);
	BOOST_CHECK_CLOSE(sdDes.position.z(), sd.position.z(), 1e-4);
}

BOOST_AUTO_TEST_CASE(binary_serialization_002_serialize_file)
{
	SensorData sd;
	sd.position = Vector3(M_PI, 2 * M_PI, 0.5 * M_PI);

	std::ofstream fileOut;
	fileOut.open("test", std::ofstream::out | std::ofstream::binary);

	dp::serialize(sd, fileOut);
	fileOut.close();

	SensorData sdRead;

	std::ifstream fileIn;
	fileIn.open("test", std::ifstream::in | std::ifstream::binary);

	sdRead = dp::deserialize<SensorData>(fileIn);
	BOOST_CHECK_EQUAL(sdRead.position, sd.position);

	fileIn.close();
}

BOOST_AUTO_TEST_CASE(binary_serialization_003_serialize_file_proto_message)
{
	LinearLocalPlannerStatus status;
	status.headingTarget = 5;
	status.currentPathSection = 32;

	std::ofstream fileOut("test_proto", std::ofstream::out | std::ofstream::binary);

	dp::serialize(status, fileOut);
	fileOut.close();

	std::ifstream fileIn("test_proto", std::ifstream::in | std::ifstream::binary);

	auto statusRead = dp::deserialize<LinearLocalPlannerStatus>(fileIn);
	BOOST_CHECK_EQUAL(statusRead.headingTarget,
			status.headingTarget);
	BOOST_CHECK_EQUAL(statusRead.currentPathSection,
			status.currentPathSection);

	fileIn.close();
}

BOOST_AUTO_TEST_CASE(binary_serialization_004_arrays)
{
	float test[] = { 1.2, 4.3, 5.2, 6.1 };
	std::ofstream fileOut("test_array", std::ofstream::out | std::ofstream::binary);

	FileToArchive to(fileOut);
	dp::serialize(to, reinterpret_cast<char*>(test), sizeof(test));
	fileOut.close();

	float testRead[4];
	std::ifstream fileIn("test_array", std::ifstream::in | std::ifstream::binary);

	FileFromArchive from(fileIn);
	dp::serialize(from, reinterpret_cast<char*>(testRead), sizeof(test));
	fileIn.close();

	BOOST_CHECK_EQUAL(test[0], testRead[0]);
	BOOST_CHECK_EQUAL(test[1], testRead[1]);
	BOOST_CHECK_EQUAL(test[2], testRead[2]);
	BOOST_CHECK_EQUAL(test[3], testRead[3]);
}

BOOST_AUTO_TEST_CASE(binary_serialization_005_int_arrays)
{
	int test[] = { 12, 13, 5, 10 };
	std::ofstream fileOut("test_array", std::ofstream::out | std::ofstream::binary);

	FileToArchive to(fileOut);
	dp::serialize(to, reinterpret_cast<char*>(test), sizeof(test));
	fileOut.close();

	int testRead[4];
	std::ifstream fileIn("test_array", std::ifstream::in | std::ifstream::binary);

	FileFromArchive from(fileIn);
	dp::serialize(from, reinterpret_cast<char*>(testRead), sizeof(test));
	fileIn.close();

	BOOST_CHECK_EQUAL(test[0], testRead[0]);
	BOOST_CHECK_EQUAL(test[1], testRead[1]);
	BOOST_CHECK_EQUAL(test[2], testRead[2]);
	BOOST_CHECK_EQUAL(test[3], testRead[3]);
}

BOOST_AUTO_TEST_SUITE_END()
