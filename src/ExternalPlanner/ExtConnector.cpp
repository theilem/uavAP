//
// Created by Mirco Theile on 29/5/25.
//
#include "uavAP/ExternalPlanner/ExtConnector.h"

#include "uavAP/Core/DataHandling/Content.hpp"

#include "cpsCore/Utilities/IDC/Header/Hash.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingIO.h"
#include "cpsCore/Utilities/IPC/IPC.h"
#include "cpsCore/Utilities/IDC/IDC.h"
#include "uavAP/Core/DataHandling/DataHandling.h"

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
            auto sens = get<ISensingIO>();
            sens->subscribeOnSensorData([this](const SensorData& data)
            {
                auto dp = get<DataPresentation>();
                Packet packet = dp->serialize(data);
                dp->addHeader(packet, Hash("sensor_data"));
                extSender_.sendPacket(packet);
            });
            auto dh = get<DataHandling<Content, Target>>();
            dh->subscribeOnData<Packet>(Content::EXTERNAL, [this](const Packet& packet)
            {
                extSender_.sendPacket(packet); // Forward packets from uavGS to external planner
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
    auto target = dp->extract<Hash>(p);
    if (target == "mission_control")
        missionControlPublisher_.publish(p);
    else if (target == "ground_station")
    {
        auto dh = get<DataHandling<Content, Target>>();
        dh->sendData<Packet>(p, Content::EXTERNAL);
    }
    else
        CPSLOG_ERROR << "ExtConnector: Unknown target in packet: " << target.hashValue;
}
