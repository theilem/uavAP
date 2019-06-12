/*
 * StaticAggregator.h
 *
 *  Created on: Jun 11, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_OBJECT_STATICAGGREGATOR_H_
#define UAVAP_CORE_OBJECT_STATICAGGREGATOR_H_

#include <uavAP/Core/Object/StaticContainer/StaticObjectContainer.h>
#include <cassert>
#include <memory>
#include <vector>

class IAggregatableObject;

template<class ...Objects>
class StaticAggregator
{
public:

	StaticAggregator();

	template<class Type>
	void
	add(std::shared_ptr<Type> obj);

	void
	add(std::shared_ptr<IAggregatableObject> obj);

	void
	add(const StaticObjectContainer<Objects...>& obj);

	template<class Type>
	std::shared_ptr<Type>
	getOne(Type* self = nullptr) const;

	template<class Type>
	std::vector<std::shared_ptr<Type> >
	getAll(Type* self = nullptr) const;

	void
	callSigHandlers(int sig);

	void
	merge(StaticAggregator& agg);

	void
	mergeInto(StaticAggregator& agg);

	/**
	 * @brief Clear the container. Will destroy all the objects.
	 */
	void
	clear();

private:

	StaticObjectContainer<Objects...> container_;

};

template<class ... Objects>
StaticAggregator<Objects...>::StaticAggregator()
{
	container_.notifyAggregationOnUpdate(*this);
}

template<class ... Objects>
template<class Type>
inline void
StaticAggregator<Objects...>::add(std::shared_ptr<Type> obj)
{
	assert(false);
}

template<class ... Objects>
inline void
StaticAggregator<Objects...>::add(std::shared_ptr<IAggregatableObject> obj)
{
	assert(false);
}

template<class ... Objects>
inline void
StaticAggregator<Objects...>::add(const StaticObjectContainer<Objects...>& obj)
{
	assert(false);
}

template<class ... Objects>
template<class Type>
inline std::shared_ptr<Type>
StaticAggregator<Objects...>::getOne(Type* self) const
{
	return container_.template getOne<Type>();
}

template<class ... Objects>
template<class Type>
inline std::vector<std::shared_ptr<Type> >
StaticAggregator<Objects...>::getAll(Type* self) const
{
	assert(false);
}

template<class ... Objects>
void
StaticAggregator<Objects...>::callSigHandlers(int sig)
{
	assert(false);
}

template<class ... Objects>
inline void
StaticAggregator<Objects...>::merge(StaticAggregator& agg)
{
	assert(false);
} //Not available

template<class ... Objects>
inline void
StaticAggregator<Objects...>::mergeInto(StaticAggregator& agg)
{
	assert(false);
} //Not available

/**
 * @brief Clear the container. Will destroy all the objects.
 */
template<class ... Objects>
inline void
StaticAggregator<Objects...>::clear()
{
	assert(false);
} //Not available

#endif /* UAVAP_CORE_OBJECT_STATICAGGREGATOR_H_ */
