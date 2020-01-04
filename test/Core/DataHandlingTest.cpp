/**
 * DataHandlingTest.cpp
 *
 *  Created on: Feb 15, 2019
 *      Author: mirco
 */

#include <boost/test/unit_test.hpp>
#include <uavAP/Core/DataPresentation/Content.h>
#include <uavAP/Core/EnumMap.hpp>
#include <uavAP/Core/Framework/Helper.h>
#include <uavAP/Core/IPC/Publisher.h>
#include <uavAP/Core/Runner/SimpleRunner.h>
#include <uavAP/Core/Scheduler/MicroSimulator.h>
#include <uavAP/Core/SensorData.h>
#include <uavAP/Core/DataHandling/DataHandling.h>
#include <uavAP/Core/IPC/IPC.h>
#include <uavAP/Core/DataPresentation/DataPresentation.h>
#include <functional>

namespace
{
class DataHandlingTestHelper: public Helper
{
public:
	DataHandlingTestHelper()
	{
		addDefaultCreator<MicroSimulator>();
		addCreator<DataHandling>();
		addDefaultCreator<IPC>();
		addDefaultCreator<DataPresentation>();
	}
};

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
	BOOST_CHECK_EQUAL(data.position.x(), 6);
	BOOST_CHECK_EQUAL(data.position.y(), 2);
	BOOST_CHECK_EQUAL(data.position.z(), 1);
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
	BOOST_CHECK_EQUAL(data.position.x(), 5);
	BOOST_CHECK_EQUAL(data.position.y(), 2);
	BOOST_CHECK_EQUAL(data.position.z(), 1);
}

}

BOOST_AUTO_TEST_CASE(data_handling_test001)
{
	DataHandlingTestHelper helper;
	Aggregator agg = helper.createAggregation("Core/config/datahandling1.json");

	auto dataHandling = agg.getOne<DataHandling>();
	auto ipc = agg.getOne<IPC>();
	auto dp = agg.getOne<DataPresentation>();

//	dataHandling->addStatusFunction(std::function<SensorData()>(statusFunction));
	dataHandling->addStatusFunction<SensorData>(std::bind(statusFunction), Content::SENSOR_DATA);

	dataHandling->subscribeOnCommand(Content::SENSOR_DATA, std::function<void
	(const SensorData&)>(commandFunction));
	auto pub = ipc->publishPackets("comm_to_flight_control");

	SimpleRunner runner(agg);

	BOOST_REQUIRE(!runner.runAllStages());
	ipc->subscribeOnPackets("flight_control_to_comm",
			std::bind(onPacket, std::placeholders::_1, dp));

	auto scheduler = agg.getOne<MicroSimulator>();

	SensorData data;
	data.position =
	{	6,2,1};
	Packet testPacket = dp->serialize(data);
	dp->addHeader(testPacket, Content::SENSOR_DATA);
	scheduler->schedule(std::bind(static_cast<void
	(Publisher<Packet>::*)(const Packet&)>(&Publisher<Packet>::publish), &pub, testPacket), Milliseconds(50));

	scheduler->simulate(Seconds(10));

	std::this_thread::sleep_for(Milliseconds(10));

}
