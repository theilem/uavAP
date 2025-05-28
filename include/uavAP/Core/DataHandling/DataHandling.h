/*
 * DataHandling.h
 *
 *  Created on: Feb 13, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_DATAHANDLING_H_
#define UAVAP_CORE_DATAHANDLING_H_

#include <map>

#include <cpsCore/cps_object>
#include <cpsCore/Utilities/DataPresentation/DataPresentation.h>
#include <cpsCore/Utilities/IPC/IPC.h>
#include <cpsCore/Utilities/Scheduler/IScheduler.h>
#include <cpsCore/Utilities/IDC/IDCSender.h>

#include "cpsCore/Utilities/IDC/IDC.h"
#include "uavAP/Core/DataHandling/Content.hpp"

template <typename T>
struct DataHandlingParams
{
    Parameter<int> period = {100, "period", true};
    Parameter<bool> useIPC = {true, "use_ipc", false};
    Parameter<T> target = {getDefaultTarget<T>(), "target", true};
    Parameter<bool> useIDC = {false, "use_idc", false};
    Parameter<std::string> idcTarget = {"autopilot", "idc_target", false};

    //Adaptive period scheduling
    Parameter<bool> useAdaptivePeriod = {false, "use_adaptive_period", false};
    Parameter<int> minPeriod = {100, "min_period", false};
    Parameter<int> maxPeriod = {1000, "max_period", false};
    Parameter<FloatingType> increment = {1.1, "increment", false};
    Parameter<FloatingType> decrement = {0.99, "decrement", false};

    template <typename Config>
    void
    configure(Config& c)
    {
        c & period;
        c & target;
        c & useIPC;
        c & useIDC;
        c & idcTarget;

        c & useAdaptivePeriod;
        c & minPeriod;
        c & maxPeriod;
        c & increment;
        c & decrement;
    }
};

template <typename C, typename T>
class DataHandling : public AggregatableObject<DataPresentation, IScheduler, IPC, IDC>,
                     public ConfigurableObject<DataHandlingParams<T>>,
                     public IRunnableObject
{
public:
    using ContentType = C;
    using TargetType = T;
    using ContentArg = std::conditional_t<std::is_arithmetic_v<ContentType>, ContentType, const ContentType&>;
    using TargetArg = std::conditional_t<std::is_arithmetic_v<TargetType>, TargetType, const TargetType&>;

    static constexpr TypeId typeId = "data_handling";

    DataHandling() = default;

    template <typename Type>
    void
    subscribeOnData(ContentArg content, std::function<void(const Type&)> commandFunc)
    {
        auto func = std::bind(&DataHandling::forwardCommand<Type>, this, std::placeholders::_1,
                              commandFunc);
        if (const auto it = subscribers_.find(content); it != subscribers_.end())
        {
            it->second.push_back(func);
            return;
        }

        std::vector<std::function<void(const Packet&)>> vec;
        vec.push_back(func);
        subscribers_.insert(std::make_pair(content, vec));
    }

    template <typename Type>
    void
    subscribeOnMemberData(const std::string& memberId, std::function<void(const Type&)> commandFunc)
    {
        auto func = std::bind(&DataHandling::forwardCommand<Type>, this, std::placeholders::_1,
                              commandFunc);
        if (const auto it = memberSubscribers_.find(memberId); it != memberSubscribers_.end())
        {
            CPSLOG_ERROR << "Member with id " << memberId << " already exists. Cannot add another one with same ID.";
            return;
        }

        memberSubscribers_.insert(std::make_pair(memberId, func));
    }

    template <typename Type>
    void
    addStatusFunction(std::function<Type ()> statusFunc, ContentArg content)
    {
        auto func = [this, statusFunc, content]()
        {
            return createPacket<Type>(statusFunc, content);
        };
        statusPackaging_.push_back(func);
    }

    template <typename Type, typename TriggerType>
    void
    addTriggeredStatusFunction(const std::function<Optional<Type> (const TriggerType&)>& statusFunc,
                               ContentArg statusContent,
                               ContentArg triggerCommand)
    {
        auto func = [this, statusFunc, statusContent](const TriggerType& trigger)
        {
            evaluateTrigger<Type, TriggerType>(trigger, statusFunc, statusContent);
        };
        this->template subscribeOnData<TriggerType>(triggerCommand, func);
    }

    template <class Object>
    void
    addConfig(Object* obj, ContentArg configContent, const std::function<void()>& callback)
    {
        subscribeOnData<ContentType>(getConfigRequest<ContentType>(), [this, configContent, obj](ContentArg content)
        {
            getConfig<Object>(obj, configContent, content);
        });
        subscribeOnData<typename Object::ParamType>(configContent, [obj, callback](const auto& p)
        {
            obj->setParams(p);
            callback();
        });
    }

    template <class Object>
    void
    addConfig(Object* obj, ContentArg configContent)
    {
        subscribeOnData<ContentType>(getConfigRequest<ContentType>(), [this, obj, configContent](ContentArg content)
        {
            getConfig<Object>(obj, configContent, content);
        });
        subscribeOnData<typename Object::ParamType>(configContent, [obj](const auto& p)
        {
            obj->setParams(p);
        });
    }

    template <class ParameterSet>
    void
    addMember(ParameterSet* member, const std::string& memberId, const std::function<void()>& callback)
    {
        subscribeOnMemberData<ParameterSet>(memberId, [member, callback](const auto& p)
        {
            *member = p;
            callback();
        });
    }


    template <typename Type>
    void
    sendData(const Type& data, ContentArg content, std::optional<TargetType> target = std::nullopt)
    {
        auto dp = get<DataPresentation>();
        if (!dp)
        {
            CPSLOG_ERROR << "Data Presentation Missing, cannot create packet.";
            return;
        }

        Packet packet = dp->serialize(data);
        dp->addHeader(packet, content);
        if (target)
            dp->addHeader(packet, *target);
        publish(packet);
    }

    void
    subscribeOnPackets(const std::function<void(const Packet&)>& packetSub);

    bool
    run(RunStage stage) override;

private:
    void
    onPacket(const Packet& packet);

    void
    sendStatus();

    template <typename Type>
    Packet
    createPacket(std::function<Type ()> statusFunc, ContentType content)
    {
        auto status = statusFunc();
        auto dp = get<DataPresentation>();
        if (!dp)
        {
            CPSLOG_ERROR << "Data Presentation Missing, cannot create packet.";
            return Packet();
        }
        auto packet = dp->serialize(status);
        dp->addHeader(packet, content);
        return packet;
    }

    template <typename Type>
    void
    forwardCommand(const Packet& packet, std::function<void (const Type&)> callback)
    {
        auto dp = get<DataPresentation>();
        callback(dp->template deserialize<Type>(packet));
    }

    template <typename Type, typename TriggerType>
    void
    evaluateTrigger(const TriggerType& trigger, std::function<Optional<Type> (const TriggerType&)> callback,
                    ContentArg statusContent)
    {
        auto status = callback(trigger);
        if (status)
        {
            auto dp = get<DataPresentation>();
            if (!dp)
            {
                CPSLOG_ERROR << "Data Presentation Missing, cannot create packet.";
                return;
            }
            auto packet = dp->serialize(*status);
            dp->addHeader(packet, statusContent);
            publish(packet);
        }
    }

    template <class Object>
    void
    getConfig(Object* obj, ContentArg configContent, ContentArg trigger)
    {
        if (trigger != configContent)
            return;
        auto dp = get<DataPresentation>();
        if (!dp)
        {
            CPSLOG_ERROR << "Data Presentation Missing, cannot create packet.";
            return;
        }
        auto packet = dp->serialize(obj->getParams());
        dp->addHeader(packet, configContent);
        publish(packet);
    }

    template <class ParameterSet>
    void
    getMember(ParameterSet* member, const std::string& memberId, const std::string& memberRequest)
    {
        if (memberRequest != memberId)
            return;
        auto dp = get<DataPresentation>();
        if (!dp)
        {
            CPSLOG_ERROR << "Data Presentation Missing, cannot create packet.";
            return;
        }
        auto packet = dp->serialize(*member);
        dp->addHeader(packet, memberRequest);
        dp->addHeader(packet, getMemberData<ContentType>());
        publish(packet);
    }

    void
    publish(const Packet& packet);

    void
    adaptPeriod(bool queueFull);

    std::vector<std::function<Packet ()>> statusPackaging_;

    std::vector<std::function<void(const Packet&)>> packetSubscriptions_;

    std::map<Content, std::vector<std::function<void
                 (const Packet&)>>> subscribers_;

    std::map<std::string, std::function<void
                 (const Packet&)>> memberSubscribers_;

    Publisher<Packet> publisher_;
    IDCSender sender_;
    Subscription ipcSubscription_;
    Event statusEvent_;
    int currentPeriod_;
};

template <typename C, typename T>
void
DataHandling<C, T>::subscribeOnPackets(const std::function<void(const Packet&)>& packetSub)
{
    packetSubscriptions_.push_back(packetSub);
}

template <typename C, typename T>
bool
DataHandling<C, T>::run(RunStage stage)
{
    switch (stage)
    {
    case RunStage::INIT:
        {
            if (!checkIsSet<DataPresentation, IScheduler>())
            {
                CPSLOG_ERROR << "DataHandling: missing deps";
                return true;
            }

            if (this->params.useIDC())
            {
                if (!checkIsSet<IDC>())
                {
                    CPSLOG_ERROR << "DataHandling: missing deps";
                    return true;
                }

                auto idc = get<IDC>();
                sender_ = idc->createSender(this->params.idcTarget());
            }

            if (this->params.useIPC())
            {
                if (!checkIsSet<IPC>())
                {
                    CPSLOG_ERROR << "DataHandling: missing deps";
                    return true;
                }

                std::string publication = EnumMap<Target>::convert(this->params.target()) + "_to_comm";
                CPSLOG_DEBUG << "Publishing to " << publication;
                auto ipc = get<IPC>();
                IPCOptions options;
                options.multiTarget = false;
                publisher_ = ipc->publishPackets(publication, options);
            }


            break;
        }
    case RunStage::NORMAL:
        {
            auto scheduler = get<IScheduler>();
            statusEvent_ = scheduler->schedule([this]
            {
                sendStatus();
            }, Milliseconds(0), Milliseconds(this->params.period()));
            currentPeriod_ = this->params.period();
            if (this->params.useIDC())
            {
                auto idc = get<IDC>();

                idc->subscribeOnPacket(this->params.idcTarget(),
                                       std::bind(&DataHandling::onPacket, this, std::placeholders::_1));
            }

            if (this->params.useIPC())
            {
                std::string subscription = "comm_to_" + EnumMap<Target>::convert(this->params.target());
                auto ipc = get<IPC>();

                IPCOptions options;
                options.multiTarget = false;
                ipcSubscription_ = ipc->subscribeOnPackets(subscription,
                                                           std::bind(&DataHandling::onPacket, this,
                                                                     std::placeholders::_1), options);
            }


            break;
        }
    default:
        break;
    }
    return false;
}

template <typename C, typename T>
void
DataHandling<C, T>::onPacket(const Packet& packet)
{
    for (const auto& packetSub : packetSubscriptions_)
    {
        packetSub(packet);
    }
    auto dp = get<DataPresentation>();
    if (!dp)
    {
        CPSLOG_ERROR << "DataPresentation missing";
        return;
    }
    auto p = packet;
    auto content = dp->template extractHeader<ContentType>(p);

    if (content == getMemberData<ContentType>())
    {
        auto memberId = dp->template extractHeader<std::string>(p);
        auto it = memberSubscribers_.find(memberId);
        if (it == memberSubscribers_.end())
        {
            CPSLOG_WARN << "Packet with Member Data content for " << memberId
                << " received, but no subscribers";
            return;
        }
        it->second(p);
        return;
    }

    auto it = subscribers_.find(content);
    if (it == subscribers_.end())
    {
        CPSLOG_DEBUG << "Packet with content " << static_cast<int>(content)
            << " received, but no subscribers";
        return;
    }

    for (const auto& k : it->second)
    {
        k(p);
    }
}

template <typename C, typename T>
void
DataHandling<C, T>::sendStatus()
{
    for (const auto& it : statusPackaging_)
    {
        publish(it());
    }
}

template <typename C, typename T>
void
DataHandling<C, T>::publish(const Packet& packet)
{
    if (this->params.useIDC())
    {
        sender_.sendPacket(packet);
    }
    if (this->params.useIPC())
    {
        bool result = publisher_.publish(packet);
        if (this->params.useAdaptivePeriod())
            adaptPeriod(result);
    }
}

template <typename C, typename T>
void
DataHandling<C, T>::adaptPeriod(bool sendSuccess)
{
    if (sendSuccess)
        currentPeriod_ = std::floor(static_cast<FloatingType>(currentPeriod_) * this->params.decrement());
    else
        currentPeriod_ = std::ceil(static_cast<FloatingType>(currentPeriod_) * this->params.increment());
    currentPeriod_ = std::clamp(currentPeriod_, this->params.minPeriod(), this->params.maxPeriod());
    statusEvent_.changePeriod(Milliseconds(currentPeriod_));
}

using HashingDataHandling = DataHandling<std::size_t, std::size_t>;
using EnumBasedDataHandling = DataHandling<Content, Target>;


#endif /* UAVAP_CORE_DATAHANDLING_H_ */
