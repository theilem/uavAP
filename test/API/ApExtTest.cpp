/*
 * ApExtTest.cpp
 *
 *  Created on: Jul 13, 2018
 *      Author: sim
 */

#include <cpsCore/Utilities/Test/TestInfo.h>
#include <uavAP/API/ap_ext/ap_ext.h>
#include <uavAP/API/ap_ext/ApExtManager.h>
#include <cpsCore/Logging/CPSLogger.h>
#include <cpsCore/Utilities/Scheduler/IScheduler.h>
#include <thread>

TEST_CASE("General ApExtTest")
{
	setConfigPath(TestInfo::getInstance().testDir + "API/config/ap_ext.json");
	REQUIRE(ap_ext_setup() == 0);

	auto& agg = getAggregator();

	CHECK_FALSE(agg.empty());
	CHECK(agg.getOne<IScheduler>());
	CHECK(agg.getOne<ApExtManager>());
	std::this_thread::sleep_for(Milliseconds(1));
	REQUIRE(ap_ext_teardown() == 0);

	//Create and kill again
	REQUIRE(ap_ext_setup() == 0);
	std::this_thread::sleep_for(Milliseconds(1));
	REQUIRE(ap_ext_teardown() == 0);

}

TEST_CASE("LinearSensorManager in ApExt")
{
	setConfigPath(TestInfo::getInstance().testDir + "API/config/ap_ext.json");
	REQUIRE(ap_ext_setup() == 0);

	auto& agg = getAggregator();

	CHECK_FALSE(agg.empty());
	CHECK(agg.getOne<IScheduler>());
	auto apExtMan = agg.getOne<ApExtManager>();
	REQUIRE(apExtMan);

	data_sample_t dataSample = {};
	{
		auto logScope = CPSLogger::LogLevelScope(LogLevel::NONE);
		CHECK(ap_ext_sense(&dataSample) == 0);
	}

	pic_sample_t picSample = {};
	imu_sample_t imuSample = {};
	imu_sample_t intimuSample = {};
	airs_sample_t airsSample = {};

	intimuSample.imu_euler_pitch = 0.7;

	dataSample.pic_sample = &picSample;
	dataSample.imu_sample = &imuSample;
	dataSample.int_imu_sample = &intimuSample;
	dataSample.airs_sample = &airsSample;
	CHECK(ap_ext_sense(&dataSample) == 0);

	auto miscValues = apExtMan->getMiscValues();
	CHECK(miscValues.size() == 2);



	REQUIRE(ap_ext_teardown() == 0);


}
