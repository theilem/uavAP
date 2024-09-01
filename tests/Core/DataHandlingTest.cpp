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
            {5, 2, 1};

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
        CHECK(content == Content::SENSOR_DATA);
        SensorData data = dp->deserialize<SensorData>(p);
        CHECK(data.position.x() == 5);
        CHECK(data.position.y() == 2);
        CHECK(data.position.z() == 1);
    }

    void onPacketMember(const Packet& packet, std::shared_ptr<DataPresentation> dp)
    {
        auto p = packet;
        Content content = dp->extractHeader<Content>(p);
        CHECK(content == Content::MEMBER_DATA);
        int data = dp->deserialize<int>(p);
        CHECK(data == 5);
    }
}

TEST_CASE("Data Handling Test")
{
    Aggregator agg = DataHandlingTestHelper::createAggregation(
        test_info::test_dir() + "/Core/config/datahandling1.json");

    auto dataHandling = agg.getOne<DataHandling>();
    auto ipc = agg.getOne<IPC>();
    auto dp = agg.getOne<DataPresentation>();

    //	dataHandling->addStatusFunction(std::function<SensorData()>(statusFunction));
    dataHandling->addStatusFunction<SensorData>([] { return statusFunction(); }, Content::SENSOR_DATA);

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
        {6, 2, 1};
    Packet testPacket = dp->serialize(data);
    dp->addHeader(testPacket, Content::SENSOR_DATA);
    //	scheduler->schedule(std::bind(static_cast<void
    //	(Publisher<Packet>::*)(const Packet&)>(&Publisher<Packet>::publish), &pub, testPacket), Milliseconds(50));

    scheduler->schedule([&testPacket, &pub] { pub.publish(testPacket); }, Milliseconds(50));

    scheduler->simulate(Seconds(10));

    std::this_thread::sleep_for(Milliseconds(10));
}

TEST_CASE("Data Handling Test 2: Members")
{
    Aggregator agg = DataHandlingTestHelper::createAggregation(
        test_info::test_dir() + "/Core/config/datahandling1.json");

    auto dataHandling = agg.getOne<DataHandling>();
    auto ipc = agg.getOne<IPC>();
    auto dp = agg.getOne<DataPresentation>();

    struct TestObject
    {
        int testParam = 0;
        int updateCounter = 0;

        inline void
        updated()
        {
            updateCounter++;
        }
    } testObject;


    IPCOptions opt;
    opt.multiTarget = false;
    auto publisher = ipc->publishPackets("comm_to_flight_control", opt);

    SimpleRunner runner(agg);
    CHECK_FALSE(runner.runAllStages());

    Packet packet;

    ipc->subscribeOnPackets("flight_control_to_comm",
                            [&packet](const Packet& p)
                            {
                                std::cout << "Packet received: " << p.getBuffer() << std::endl;
                                packet = p;
                            }, opt);

    dataHandling->addMember(&testObject.testParam, "test_param_id", std::bind(&TestObject::updated, &testObject));

    Packet p = dp->serialize<std::string>("test_param_id");
    dp->addHeader(p, Content::REQUEST_MEMBER);
    std::cout << "Packet: " << p.getBuffer() << std::endl;

    auto scheduler = agg.getOne<MicroSimulator>();
    scheduler->schedule([&publisher, &p] { publisher.publish(p); }, Milliseconds(50));
    scheduler->simulate(Seconds(1));
    std::this_thread::sleep_for(Milliseconds(10));

    std::cout << "Packet size: " << packet.getSize() << std::endl;



    // dataHandling->addMember(&testObject.testParam, Content::PAR
}
