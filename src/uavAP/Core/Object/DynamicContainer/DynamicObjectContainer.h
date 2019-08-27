/*
 * DynamicObjectContainer.h
 *
 *  Created on: Jun 10, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_OBJECT_DYNAMICCONTAINER_DYNAMICOBJECTCONTAINER_H_
#define UAVAP_CORE_OBJECT_DYNAMICCONTAINER_DYNAMICOBJECTCONTAINER_H_
#include <memory>
#include <vector>

class IAggregatableObject;
class Aggregator;

struct DynamicObjectContainer
{
public:

	template <typename Type>
	std::shared_ptr<Type>
	getOne(Type* self) const;

	template <typename Type>
	std::vector<std::shared_ptr<Type>>
	getAll(Type* self) const;

	void
	add(std::shared_ptr<IAggregatableObject> obj);

	void
	add(const DynamicObjectContainer& obj);

	void
	notifyAggregationOnUpdate(const Aggregator& agg);

	void
	clear();

	const std::vector<std::shared_ptr<IAggregatableObject> >&
	getContainer() const;

private:

	std::vector<std::shared_ptr<IAggregatableObject> > container_;
};

template<typename Type>
inline std::shared_ptr<Type>
DynamicObjectContainer::getOne(Type* self) const
{
#ifdef ERIKA
	return nullptr;
#else
	for (auto it : container_)
	{
		if (auto ret = std::dynamic_pointer_cast<Type>(it))
		{
			if (ret.get() == self)
				continue;
			return ret;
		}
	}
	return nullptr;
#endif
}

template<class Type>
inline std::vector<std::shared_ptr<Type> >
DynamicObjectContainer::getAll(Type* self) const
{
#ifdef ERIKA
	return nullptr;
#else
	std::vector<std::shared_ptr<Type> > vec;
	for (auto it : container_)
	{
		if (auto ret = std::dynamic_pointer_cast<Type>(it))
		{
			if (ret.get() == self)
				continue;
			vec.push_back(ret);
		}
	}
	return vec;
#endif
}

#endif /* UAVAP_CORE_OBJECT_DYNAMICCONTAINER_DYNAMICOBJECTCONTAINER_H_ */
