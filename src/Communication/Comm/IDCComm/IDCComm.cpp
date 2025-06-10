/**
 *  @file         IDCComm.cpp
 *  @author Mirco Theile
 *  @date      30 July 2017
 *  @brief      UAV Autopilot Communication Serial Comm Source File
 *
 *  Description
 */

#include <uavAP/Communication/Comm/IDCComm/IDCComm.h>
#include <uavAP/Core/DataHandling/Content.hpp>
#include <cpsCore/Utilities/IDC/IDC.h>
#include <cpsCore/Utilities/IPC/IPC.h>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <cpsCore/Utilities/Scheduler/IScheduler.h>

bool
IDCComm::run(RunStage stage)
{
    switch (stage)
    {
    case RunStage::INIT:
        {
            if (!checkIsSet<IPC, IDC, DataPresentation>())
            {
                CPSLOG_ERROR << "SerialComm: missing dependency";
                return true;
            }

            auto ipc = get<IPC>();

            IPCOptions options;
            options.multiTarget = false;
            for (const auto& targetPub : params.targetPubs())
            {
                CPSLOG_DEBUG << "Publishing to target " << EnumMap<Target>::convert(targetPub.first) << " on " <<
                    targetPub.second;
                publishers_.insert(std::make_pair(targetPub.first, ipc->publishPackets(targetPub.second, options)));
            }

            auto idc = get<IDC>();
            sender_ = idc->createSender("ground_station");

            break;
        }
    case RunStage::NORMAL:
        {
            auto idc = get<IDC>();

            idcConnection_ = idc->subscribeOnPacket("ground_station", [this](const Packet& packet)
            {
                receivePacket(packet);
            });
            auto ipc = get<IPC>();

            IPCOptions options;
            options.multiTarget = false;
            options.retry = true;

            for (const auto& targetSub : params.targetSubs())
            {
                CPSLOG_DEBUG << "Subscribing to target " << EnumMap<Target>::convert(targetSub.first) << " on " <<
                    targetSub.second;
                auto sub = ipc->subscribeOnPackets(
                    targetSub.second, [this](const Packet& packet)
                    {
                        sendPacket(packet);
                    }, options);
                subscriptions_.insert(std::make_pair(targetSub.first, sub));

                if (!sub.connected())
                {
                    CPSLOG_DEBUG << EnumMap<Target>::convert(targetSub.first)
                        << " not found. Retry later.";
                }
            }
            break;
        }
    case RunStage::FINAL:
        {
            CPSLOG_DEBUG << "Run stage final";
            break;
        }
    default:
        {
            break;
        }
    }

    return false;
}

void
IDCComm::sendPacket(const Packet& packet)
{
    LockGuard lock(senderMutex_);
    sender_.sendPacket(packet);
}

void
IDCComm::receivePacket(const Packet& packet)
{
    auto dp = get<DataPresentation>();
    if (!dp)
    {
        CPSLOG_ERROR << "Data Presentation missing. Cannot handle receive.";
        return;
    }

    Packet p = packet;
    auto target = dp->extractHeader<Target>(p);

    if (target == Target::BROADCAST)
    {
        for (auto& [key, pub] : publishers_)
        {
            pub.publish(p);
        }
    }
    else
    {
        auto it = publishers_.find(target);
        if (it == publishers_.end())
        {
            CPSLOG_ERROR << "Target " << EnumMap<Target>::convert(target) << " not found in publishers.";
            return;
        }
        it->second.publish(p);
    }
}
