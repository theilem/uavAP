/*
 * DataHandling.h
 *
 *  Created on: Feb 13, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_DATAHANDLING_H_
#define UAVAP_CORE_DATAHANDLING_H_
#include <uavAP/Core/DataPresentation/Content.h>
#include <uavAP/Core/DataPresentation/ContentMapping.h>
#include <uavAP/Core/DataPresentation/IDataPresentation.h>
#include <uavAP/Core/DataPresentation/Packet.h>
#include <uavAP/Core/IPC/IPC.h>
#include <uavAP/Core/IPC/Publisher.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Runner/IRunnableObject.h>
#include <uavAP/Core/Time.h>

class IScheduler;

class DataHandling: public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "data_handling";

	DataHandling();

	static std::shared_ptr<DataHandling>
	create(const Configuration& config);

	bool
	configure(const Configuration& config);

	template<typename Type>
	void
	subscribeOnCommand(Content content, std::function<void
	(const Type&)> commandFunc);

	template<typename Type>
	void
	addStatusFunction(std::function<Type
	()> statusFunc);

	template<typename Type, typename TriggerType>
	void
	addTriggeredStatusFunction(std::function<boost::optional<Type>
	(const TriggerType&)> statusFunc, Content TriggerCommand);

	template<typename Type>
	void
	sendData(const Type& data, Content content, Target target = Target::BROADCAST);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

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
	()> statusFunc);

	template<typename Type>
	void
	forwardCommand(const boost::any& any, std::function<void
	(const Type&)> callback);

	template<typename Type, typename TriggerType>
	void
	evaluateTrigger(const TriggerType& any, std::function<boost::optional<Type>
	(const TriggerType&)> callback);

	Target target_;
	std::string targetName_;
	int period_;

	ObjectHandle<IDataPresentation<Content, Target>> dataPresentation_;
	ObjectHandle<IScheduler> scheduler_;
	ObjectHandle<IPC> ipc_;

	std::vector<std::function<Packet
	()>> statusPackaging_;

	std::unordered_map<Content, std::vector<std::function<void
	(const boost::any&)>>> subscribers_;

	Publisher publisher_;
};

template<typename Type>
inline void
DataHandling::addStatusFunction(std::function<Type
()> statusFunc)
{
	auto func = std::bind(&DataHandling::createPacket<Type>, this, statusFunc);
	statusPackaging_.push_back(func);
}

template<typename Type>
inline Packet
DataHandling::createPacket(std::function<Type
()> statusFunc)
{
	auto status = statusFunc();
	auto dp = dataPresentation_.get();
	if (!dp)
	{
		APLOG_ERROR << "Data Presentation Missing, cannot create packet.";
		return Packet();
	}
	Content content = TypeToEnum<Type, Content>::value;
	return dp->serialize(status, content);

}

template<typename Type>
inline void
DataHandling::subscribeOnCommand(Content content, std::function<void
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

	std::vector<std::function<void
	(const boost::any&)>> vec;
	vec.push_back(func);
	subscribers_.insert(std::make_pair(content, vec));
}

template<typename Type>
inline void
DataHandling::forwardCommand(const boost::any& any, std::function<void
(const Type&)> callback)
{
	callback(boost::any_cast<Type>(any));
}

template<typename Type, typename TriggerType>
inline void
DataHandling::addTriggeredStatusFunction(std::function<boost::optional<Type>
(const TriggerType&)> statusFunc, Content triggerCommand)
{
	std::function<void
	(const TriggerType&)> func = std::bind(&DataHandling::evaluateTrigger<Type, TriggerType>, this,
			std::placeholders::_1, statusFunc);
	subscribeOnCommand(triggerCommand, func);
}

template<typename Type, typename TriggerType>
inline void
DataHandling::evaluateTrigger(const TriggerType& trigger, std::function<boost::optional<Type>
(const TriggerType&)> callback)
{
	auto status = callback(trigger);
	if (status)
	{
		auto dp = dataPresentation_.get();
		if (!dp)
		{
			APLOG_ERROR << "Data Presentation Missing, cannot create packet.";
			return;
		}
		Content content = TypeToEnum<Type, Content>::value;
		publisher_.publish(dp->serialize(*status, content));
	}
}

template<typename Type>
inline void
DataHandling::sendData(const Type& data, Content content, Target target)
{
	auto dp = dataPresentation_.get();
	if (!dp)
	{
		APLOG_ERROR << "Data Presentation Missing, cannot create packet.";
		return;
	}

	Packet packet = dp->serialize(data, content);
	dp->setTarget(packet, target);
	publisher_.publish(packet);

}

#endif /* UAVAP_CORE_DATAHANDLING_H_ */
