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
#include <cpsCore/Utilities/IDC/IDCSender.h>

#include "uavAP/Core/DataHandling/DataHandlingParams.h"
#include "uavAP/Core/DataHandling/Content.hpp"

class IScheduler;

class DataHandling : public AggregatableObject<DataPresentation, IScheduler, IPC, IDC>,
					 public ConfigurableObject<DataHandlingParams>,
					 public IRunnableObject
{
public:

	static constexpr TypeId typeId = "data_handling";

	DataHandling() = default;

	template<typename Type>
	void
	subscribeOnData(Content content, std::function<void
			(const Type&)> commandFunc);

	template<typename Type>
	void
	addStatusFunction(std::function<Type
			()> statusFunc, Content content);

	template<typename Type, typename TriggerType>
	void
	addTriggeredStatusFunction(std::function<Optional<Type>
			(const TriggerType&)> statusFunc, Content statusContent, Content TriggerCommand);

	template<class Object>
	void
	addConfig(Object* obj, Content configContent, const std::function<void(void)>& callback);

	template<class Object>
	void
	addConfig(Object* obj, Content configContent);

	template<class ParameterSet>
	void
	addMember(ParameterSet* member, Content content, const std::function<void(void)>& callback);

	template<class ParameterSet>
	void
	addMember(ParameterSet* member, Content content);

	template<typename Type>
	void
	sendData(const Type& data, Content content, Target target = Target::BROADCAST);

	void
	subscribeOnPackets(const std::function<void(const Packet&)>& packetSub);

	bool
	run(RunStage stage) override;

private:

	void
	onPacket(const Packet& packet);

	void
	sendStatus();

	template<typename Type>
	Packet
	createPacket(std::function<Type
			()> statusFunc, Content content);

	template<typename Type>
	void
	forwardCommand(const Packet& packet, std::function<void
			(const Type&)> callback);

	template<typename Type, typename TriggerType>
	void
	evaluateTrigger(const TriggerType& trigger, std::function<Optional<Type>
			(const TriggerType&)> callback, Content statusContent);


	template<class Object>
	void
	getConfig(Object* obj, Content configContent, Content trigger);

	template<class ParameterSet>
	void
	getMember(ParameterSet* member, Content content, Content trigger);

	void
	publish(const Packet& packet);

	std::vector<std::function<Packet
			()>> statusPackaging_;

	std::vector<std::function<void(const Packet&)>> packetSubscriptions_;

	std::map<Content, std::vector<std::function<void
			(const Packet&)>>> subscribers_;

	Publisher<Packet> publisher_;
	IDCSender sender_;
	Subscription ipcSubscription_;
};

template<typename Type>
inline void
DataHandling::addStatusFunction(std::function<Type
		()> statusFunc, Content content)
{
	auto func = std::bind(&DataHandling::createPacket<Type>, this, statusFunc, content);
	statusPackaging_.push_back(func);
}

template<typename Type>
inline Packet
DataHandling::createPacket(std::function<Type
		()> statusFunc, Content content)
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

template<typename Type>
inline void
DataHandling::subscribeOnData(Content content, std::function<void
		(const Type&)> commandFunc)
{
	auto func = std::bind(&DataHandling::forwardCommand<Type>, this, std::placeholders::_1,
						  commandFunc);
	auto it = subscribers_.find(content);
	if (it != subscribers_.end())
	{
		it->second.push_back(func);
		return;
	}

	std::vector<std::function<void(const Packet&)>> vec;
	vec.push_back(func);
	subscribers_.insert(std::make_pair(content, vec));
}

template<typename Type>
inline void
DataHandling::forwardCommand(const Packet& packet, std::function<void
		(const Type&)> callback)
{
	auto dp = get<DataPresentation>();
	callback(dp->deserialize<Type>(packet));
}

template<typename Type, typename TriggerType>
inline void
DataHandling::addTriggeredStatusFunction(std::function<Optional<Type>
		(const TriggerType&)> statusFunc, Content statusContent, Content triggerCommand)
{
	std::function<void
			(const TriggerType&)> func = std::bind(&DataHandling::evaluateTrigger<Type, TriggerType>, this,
												   std::placeholders::_1, statusFunc, statusContent);
	subscribeOnData(triggerCommand, func);
}


template<class Object>
void
DataHandling::addConfig(Object* obj, Content configContent, const std::function<void(void)>& callback)
{
	std::function<void
			(const Content&)> func = std::bind(&DataHandling::getConfig<Object>, this,
											   obj, configContent, std::placeholders::_1);
	subscribeOnData(Content::REQUEST_CONFIG, func);
	subscribeOnData<typename Object::ParamType>(configContent, [obj, callback](const auto& p)
	{
		obj->setParams(p);
		callback();
	});

}


template<class Object>
void
DataHandling::addConfig(Object* obj, Content configContent)
{
	subscribeOnData<Content>(Content::REQUEST_CONFIG, [this, obj, configContent](const Content& content)
	{ getConfig<Object>(obj, configContent, content); });
	subscribeOnData<typename Object::ParamType>(configContent, [obj](const auto& p)
	{ obj->setParams(p); });

}

template<class ParameterSet>
void
DataHandling::addMember(ParameterSet *member, Content content, const std::function<void()> &callback)
{
	std::function<void
		(const Content&)> func = std::bind(&DataHandling::getMember<ParameterSet>, this,
										   member, content, std::placeholders::_1);
	subscribeOnData(Content::REQUEST_MEMBER, func);
	subscribeOnData<ParameterSet>(content, [member, callback](const auto& p)
	{
		*member = p;
		callback();
	});

}

template<class ParameterSet>
void
DataHandling::addMember(ParameterSet *member, Content content)
{
	std::function<void
		(const Content&)> func = std::bind(&DataHandling::getMember<ParameterSet>, this,
										   member, content, std::placeholders::_1);
	subscribeOnData(Content::REQUEST_MEMBER, func);
	subscribeOnData<ParameterSet>(content, [member](const auto& p)
	{
		*member = p;
	});

}

template<class Object>
void
DataHandling::getConfig(Object* obj, Content configContent, Content trigger)
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


template<class ParameterSet>
void
DataHandling::getMember(ParameterSet* member, Content content, Content trigger)
{
	if (trigger != content)
		return;
	auto dp = get<DataPresentation>();
	if (!dp)
	{
		CPSLOG_ERROR << "Data Presentation Missing, cannot create packet.";
		return;
	}
	auto packet = dp->serialize(*member);
	dp->addHeader(packet, content);
	publish(packet);
}


template<typename Type, typename TriggerType>
inline void
DataHandling::evaluateTrigger(const TriggerType& trigger, std::function<Optional<Type>
		(const TriggerType&)> callback, Content statusContent)
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

template<typename Type>
inline void
DataHandling::sendData(const Type& data, Content content, Target target)
{
	auto dp = get<DataPresentation>();
	if (!dp)
	{
		CPSLOG_ERROR << "Data Presentation Missing, cannot create packet.";
		return;
	}

	Packet packet = dp->serialize(data);
	dp->addHeader(packet, content);
	dp->addHeader(packet, target);
	publish(packet);

}

#endif /* UAVAP_CORE_DATAHANDLING_H_ */
