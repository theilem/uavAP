//
// Created by Mirco Theile on 29/5/25.
//
#include "uavAP/ExternalPlanner/ExtConnector.h"

#include "uavAP/Core/DataHandling/Content.hpp"

#include "cpsCore/Utilities/IDC/Header/HashingHeader.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingIO.h"
#include "cpsCore/Utilities/IPC/IPC.h"
#include "cpsCore/Utilities/IDC/IDC.h"

bool
ExtConnector::run(RunStage stage)
{
    switch (stage)
    {
    case RunStage::INIT:
        {
            if (!checkIsSetAll())
            {
                CPSLOG_ERROR << "ExtConnector: Dependency missing";
                return true;
            }
            auto ipc = get<IPC>();
            missionControlPublisher_ = ipc->publishPackets("external_to_mission_control");
            commsPublisher_ = ipc->publishPackets(EnumMap<Target>::convert(Target::EXTERNAL) + "_to_comm");
            break;
        }
    case RunStage::NORMAL:
        {
            auto idc = get<IDC>();
            extSender_ = idc->createSender("external_planner");
            idc->subscribeOnPacket("external_planner", [this](const Packet& packet)
            {
                onExtPacket(packet);
            });
            auto ipc = get<IPC>();
            ipc->subscribeOnPackets("comm_to_" + EnumMap<Target>::convert(Target::EXTERNAL),
                                    [this](const Packet& packet)
                                    {
                                        extSender_.sendPacket(packet);
                                    });
            auto sens = get<ISensingIO>();
            sens->subscribeOnSensorData([this](const SensorData& data)
            {
                std::cout << "ExtConnector: Sending sensor data to external planner." << std::endl;
                auto dp = get<DataPresentation>();
                Packet packet = dp->serialize(data);
                HashingHeader header("sensor_data");
                dp->addHeader(packet, header);
                extSender_.sendPacket(packet);
            });
        }
    default:
        break;
    }
    return false;
}

void
ExtConnector::onExtPacket(const Packet& packet)
{
    auto p = packet;
    auto dp = get<DataPresentation>();
    auto target = dp->extract<Target>(p);
    if (target == Target::MISSION_CONTROL)
        missionControlPublisher_.publish(p);
    else if (target == Target::COMMUNICATION)
        commsPublisher_.publish(p);
    else
        CPSLOG_ERROR << "ExtConnector: Unknown target in packet: " << EnumMap<Target>::convert(target);
}
