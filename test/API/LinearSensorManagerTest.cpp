/*
 * ApExtTest.cpp
 *
 *  Created on: Jul 13, 2018
 *      Author: sim
 */

#include <cpsCore/Utilities/Test/TestInfo.h>
#include <uavAP/API/ap_ext/LinearSensorManager.h>

TEST_CASE("Test LinearSensor")
{
	LinearSensor sensor;
	LinearSensorParams sensorParams;
	sensorParams.channel = 5;
	sensorParams.offset = 2.0;
	sensorParams.slope = 3.0;
	sensor.setParams(sensorParams);

	unsigned short channelArray1[10] = {};
	unsigned short channelArray2[5] = {};

	channelArray1[5] = 7;

	auto logScope = CPSLogger::LogLevelScope(LogLevel::NONE);

	CHECK(sensor.getValue(channelArray1) == 23.0);
	CHECK(sensor.getValue(channelArray2) == -1.0);
}

TEST_CASE("Test LinearSensorManager Get Values")
{
	LinearSensorParams sensorParams1;
	sensorParams1.channel = 5;
	sensorParams1.offset = 2.0;
	sensorParams1.slope = 3.0;

	LinearSensorParams sensorParams2;
	sensorParams2.channel = 3;
	sensorParams2.offset = 5.5;
	sensorParams2.slope = -12.2;

	LinearSensorManager sensorMan;
	LinearSensorManagerParams sensorManParams;
	sensorManParams.sensors().insert(std::make_pair("sensor_1", sensorParams1));
	sensorManParams.sensors().insert(std::make_pair("sensor_2", sensorParams2));
	sensorMan.setParams(sensorManParams);

	CHECK_FALSE(sensorMan.run(RunStage::INIT));

	unsigned short channelArray1[10] = {};
	unsigned short channelArray2[5] = {};

	channelArray1[5] = 7;
	channelArray1[3] = 1856;

	channelArray2[3] = 72;

	auto vals = sensorMan.getValues(channelArray1);
	REQUIRE(vals.size() == 2);
	REQUIRE_NOTHROW(vals.at("sensor_1"));
	REQUIRE_NOTHROW(vals.at("sensor_2"));
	CHECK_THROWS(vals.at("random_wrong_sensor_name"));

	CHECK(vals.at("sensor_1") == 23.0);
	CHECK(vals.at("sensor_2") == Approx(-22637.7));


	auto logScope = CPSLogger::LogLevelScope(LogLevel::NONE);
	vals = sensorMan.getValues(channelArray2);
	REQUIRE_NOTHROW(vals.at("sensor_1"));
	REQUIRE_NOTHROW(vals.at("sensor_2"));
	CHECK_THROWS(vals.at("random_wrong_sensor_name"));

	CHECK(vals.at("sensor_1") == -1.0);
	CHECK(vals.at("sensor_2") == Approx(-872.9));

}
