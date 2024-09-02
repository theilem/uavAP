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

#include <boost/interprocess/ipc/message_queue.hpp>

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

TEST_CASE("Data Handling Adaptive Period")
{
    CPSLogger::LogLevelScope logLevel(LogLevel::ERROR);
    auto agg = DataHandlingTestHelper::createAggregation(
        test_info::test_dir() + "/Core/config/datahandling2.json");

    SimpleRunner runner(agg);
    CHECK_FALSE(runner.runStage(RunStage::INIT));
    auto dh = agg.getOne<DataHandling>();
    auto scheduler = agg.getOne<MicroSimulator>();
    scheduler->setMainThread();
    auto statusMessageTiming = std::vector<int>();
    dh->addStatusFunction<SensorData>([scheduler, &statusMessageTiming]()
    {
        statusMessageTiming.push_back(std::chrono::duration_cast<Milliseconds>(scheduler->timeSinceStart()).count());
        return SensorData();
    }, Content::SENSOR_DATA);
    CHECK_FALSE(runner.runStage(RunStage::NORMAL));
    //Status Events: 0, 100, 200, 300, 400, 500, 600, 700, 800, 900, (1000 full) -> period 150
    //Status Events: 1150 -> period 225, 1375 -> period 338, 1713 -> period 507
    //Status Events: 2220 -> period 761, 2981 -> period 1000, 3981 -> period 1000
    scheduler->simulate(Seconds(4));
    CHECK(statusMessageTiming.size() == 17);
    CHECK(statusMessageTiming[0] == 0);
    CHECK(statusMessageTiming[1] == 100);
    CHECK(statusMessageTiming[2] == 200);
    CHECK(statusMessageTiming[3] == 300);
    CHECK(statusMessageTiming[4] == 400);
    CHECK(statusMessageTiming[5] == 500);
    CHECK(statusMessageTiming[6] == 600);
    CHECK(statusMessageTiming[7] == 700);
    CHECK(statusMessageTiming[8] == 800);
    CHECK(statusMessageTiming[9] == 900);
    CHECK(statusMessageTiming[10] == 1000);
    CHECK(statusMessageTiming[11] == 1150);
    CHECK(statusMessageTiming[12] == 1375);
    CHECK(statusMessageTiming[13] == 1713);
    CHECK(statusMessageTiming[14] == 2220);
    CHECK(statusMessageTiming[15] == 2981);
    CHECK(statusMessageTiming[16] == 3981);
    statusMessageTiming.clear();

    auto messageQueue = boost::interprocess::message_queue(boost::interprocess::open_only, "flight_control_to_comm");
    CHECK(messageQueue.get_num_msg() == 10);
    boost::interprocess::message_queue::size_type recvdSize;
    unsigned int priority;
    std::vector<char> message;
    message.resize(messageQueue.get_max_msg_size());
    for (int i = 0; i < 10; i++)
        messageQueue.receive(message.data(), message.size(), recvdSize, priority);

    //Status Event Periods: 1000, 800, 640, 512, 409, 327,      261, 208, 166, 132
    //Status Event Timings: 4981, 5781, 6421, 6933, 7342, 7669, 7930, 8138, 8304, 8436
    scheduler->simulate(Milliseconds(4500));
    CHECK(statusMessageTiming.size() == 10);
    CHECK(statusMessageTiming[0] == 4981);
    CHECK(statusMessageTiming[1] == 5781);
    CHECK(statusMessageTiming[2] == 6421);
    CHECK(statusMessageTiming[3] == 6933);
    CHECK(statusMessageTiming[4] == 7342);
    CHECK(statusMessageTiming[5] == 7669);
    CHECK(statusMessageTiming[6] == 7930);
    CHECK(statusMessageTiming[7] == 8138);
    CHECK(statusMessageTiming[8] == 8304);
    CHECK(statusMessageTiming[9] == 8436);


}
