/**
 * DataHandlingTest.cpp
 *
 *  Created on: Feb 15, 2019
 *      Author: mirco
 */

#include <cpsCore/Utilities/Test/TestInfo.h>
#include <uavAP/Core/DataHandling/Content.hpp>
#include <uavAP/Core/SensorData.h>
#include <uavAP/Core/DataHandling/DataHandling.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <functional>
#include <cpsCore/Utilities/Scheduler/MicroSimulator.h>
#include <cpsCore/Synchronization/SimpleRunner.h>
#include <cpsCore/Framework/StaticHelper.h>
#include <cpsCore/Utilities/Scheduler/SchedulerFactory.h>

namespace
{

using DataHandlingTestDefaultHelper = StaticHelper<MicroSimulator,
		DataHandling,
		DataPresentation>;

using DataHandlingTestHelper = StaticHelper<DataHandlingTestDefaultHelper,
		IPC
		>;

SensorData
statusFunction()
{
	std::this_thread::sleep_for(std::chrono::milliseconds(1));
	SensorData data;
	data.position =
	{	5,2,1};

	return data;
}

void
commandFunction(const SensorData& data)
{
	CHECK(data.position.x() == 6);
	CHECK(data.position.y() == 2);
	CHECK(data.position.z() == 1);
}
//
//void
//sendPacket(const Packet& packet)

void
onPacket(const Packet& packet, std::shared_ptr<DataPresentation> dp)
{
	auto p = packet;
	Content content = dp->extractHeader<Content>(p);
	SensorData data = dp->deserialize<SensorData>(p);
	CHECK(data.position.x() == 5);
	CHECK(data.position.y() == 2);
	CHECK(data.position.z() == 1);
}

}

TEST_CASE("Data Handling Test")
{
	Aggregator agg = DataHandlingTestHelper::createAggregation(test_info::test_dir() + "/Core/config/datahandling1.json");

	auto dataHandling = agg.getOne<DataHandling>();
	auto ipc = agg.getOne<IPC>();
	auto dp = agg.getOne<DataPresentation>();

//	dataHandling->addStatusFunction(std::function<SensorData()>(statusFunction));
	dataHandling->addStatusFunction<SensorData>(std::bind(statusFunction), Content::SENSOR_DATA);

	dataHandling->subscribeOnData(Content::SENSOR_DATA, std::function<void
	(const SensorData&)>(commandFunction));
	IPCOptions options;
	options.multiTarget = false;
	auto pub = ipc->publishPackets("comm_to_flight_control", options);

	SimpleRunner runner(agg);

	CHECK(!runner.runAllStages());
	ipc->subscribeOnPackets("flight_control_to_comm",
			std::bind(onPacket, std::placeholders::_1, dp), options);

	auto scheduler = agg.getOne<MicroSimulator>();

	SensorData data;
	data.position =
	{	6,2,1};
	Packet testPacket = dp->serialize(data);
	dp->addHeader(testPacket, Content::SENSOR_DATA);
//	scheduler->schedule(std::bind(static_cast<void
//	(Publisher<Packet>::*)(const Packet&)>(&Publisher<Packet>::publish), &pub, testPacket), Milliseconds(50));

	scheduler->schedule([&testPacket, &pub]{pub.publish(testPacket);}, Milliseconds(50));

	scheduler->simulate(Seconds(10));

	std::this_thread::sleep_for(Milliseconds(10));

}
