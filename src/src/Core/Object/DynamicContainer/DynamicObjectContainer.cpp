/*
 * DynamicObjectContainer.cpp
 *
 *  Created on: Jun 10, 2019
 *      Author: mirco
 */
#include <uavAP/Core/Object/Aggregator.h>
#include <uavAP/Core/Object/DynamicContainer/DynamicObjectContainer.h>
#include <uavAP/Core/Object/IAggregatableObject.h>

void
DynamicObjectContainer::clear()
{
	container_.clear();
}

void
DynamicObjectContainer::add(std::shared_ptr<IAggregatableObject> obj)
{
	container_.push_back(obj);
}

void
DynamicObjectContainer::notifyAggregationOnUpdate(const Aggregator& agg)
{
	for (auto it : container_)
	{
		it->notifyAggregationOnUpdate(agg);
	}
}

void
DynamicObjectContainer::add(const DynamicObjectContainer& obj)
{
	for (auto it : obj.getContainer())
	{
		container_.push_back(it);
	}
}

const std::vector<std::shared_ptr<IAggregatableObject> >&
DynamicObjectContainer::getContainer() const
{
	return container_;
}
