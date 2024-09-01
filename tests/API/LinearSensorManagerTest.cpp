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

TEST_CASE("Test LinearSensorManager Filtering")
{
	LinearSensorParams sensorParams;
	sensorParams.channel = 5;
	sensorParams.offset = 2.0;
	sensorParams.slope = 3.0;

	LinearSensorParams sensorParamsWithFilter;
	sensorParamsWithFilter.channel = 3;
	sensorParamsWithFilter.offset = 5.5;
	sensorParamsWithFilter.slope = -12.2;

	Control::LowPassFilterParams filterParams;
	filterParams.samplingPeriod = 10;
	filterParams.cutOffFrequency = 30;

	sensorParamsWithFilter.filter = filterParams;

	LinearSensorParams sensorParamsWithMinMaxFilter;
	sensorParamsWithMinMaxFilter.channel = 4;
	sensorParamsWithMinMaxFilter.offset = -2.0;
	sensorParamsWithMinMaxFilter.slope = 3.0;

	Control::LowPassFilterParams minMaxFilterParams;
	minMaxFilterParams.samplingPeriod = 10;
	minMaxFilterParams.cutOffFrequency = 30;
	minMaxFilterParams.minValue = 0;
	minMaxFilterParams.maxValue = 10;

	sensorParamsWithMinMaxFilter.filter = minMaxFilterParams;

	LinearSensorManager sensorMan;
	LinearSensorManagerParams sensorManParams;
	sensorManParams.sensors().insert(std::make_pair("sensor", sensorParams));
	sensorManParams.sensors().insert(std::make_pair("sensor_with_filter", sensorParamsWithFilter));
	sensorManParams.sensors().insert(std::make_pair("sensor_with_minmax_filter", sensorParamsWithMinMaxFilter));
	sensorMan.setParams(sensorManParams);

	CHECK_FALSE(sensorMan.run(RunStage::INIT));

	unsigned short channelArray1[10] = {};
	unsigned short channelArray2[5] = {};
	unsigned short channelArray3[5] = {};

	channelArray1[5] = 7;
	channelArray1[4] = 3;
	channelArray1[3] = 1856;

	channelArray2[3] = 72;
	channelArray2[4] = 0;

	channelArray3[4] = 72;

	auto vals = sensorMan.getValues(channelArray1);
	REQUIRE(vals.size() == 3);
	REQUIRE_NOTHROW(vals.at("sensor"));
	REQUIRE_NOTHROW(vals.at("sensor_with_filter"));
	REQUIRE_NOTHROW(vals.at("sensor_with_minmax_filter"));

	CHECK(vals.at("sensor") == 23.0);
	CHECK(vals.at("sensor_with_filter") == Approx(-5867.3));
	CHECK(vals.at("sensor_with_minmax_filter") == Approx(1.81).epsilon(0.1));


	auto logScope = CPSLogger::LogLevelScope(LogLevel::NONE);
	vals = sensorMan.getValues(channelArray2);
	REQUIRE_NOTHROW(vals.at("sensor"));
	REQUIRE_NOTHROW(vals.at("sensor_with_filter"));
	REQUIRE_NOTHROW(vals.at("sensor_with_minmax_filter"));

	CHECK(vals.at("sensor") == -1.0);
	CHECK(vals.at("sensor_with_filter") == Approx(-4572.8));
	CHECK(vals.at("sensor_with_minmax_filter") == Approx(1.81).epsilon(0.1));

	vals = sensorMan.getValues(channelArray3);
	CHECK(vals.at("sensor_with_minmax_filter") == Approx(1.81).epsilon(0.1));

}
