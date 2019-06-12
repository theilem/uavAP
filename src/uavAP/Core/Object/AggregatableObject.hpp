/*
 * AggregatableObject.hpp
 *
 *  Created on: Jun 7, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_OBJECT_AGGREGATABLEOBJECT_HPP_
#define UAVAP_CORE_OBJECT_AGGREGATABLEOBJECT_HPP_
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Object/ObjectHandleContainer.hpp>

template<class ... Objects>
class AggregatableObject: public IAggregatableObject
{
public:

	template<class Type>
	using PtrType = std::shared_ptr<Type>;
	template<class Type>
	using WeakPtrType = std::weak_ptr<Type>;

	template <class Agg>
	void
	notifyAggregationOnUpdate(const Agg& agg);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	template<class Ret>
	PtrType<Ret>
	get();

	template<class Ret>
	bool
	isSet() const;

protected:

	ObjectHandleContainer<Objects...> container_;

};
//
//template<class ... Objects>
//inline void
//AggregatableObject<Objects...>::notifyAggregationOnUpdate(const Aggregator& agg)
//{
//	container_.setFromAggregationIfNotSet(agg);
//}
//
//template<class ... Objects>
//template <class Agg>
//void
//AggregatableObject<Objects...>::notifyAggregationOnUpdate(const Agg& agg)
//{
//	container_.setFromAggregationIfNotSet(agg);
//}
//
//template<class ... Objects>
//template<class Ret>
//inline AggregatableObject<Objects...>::PtrType<Ret>
//AggregatableObject<Objects...>::get()
//{
//	return container_.template get<Ret>();
//}
//
//template<class ... Objects>
//template<class Ret>
//inline bool
//AggregatableObject<Objects...>::isSet() const
//{
//	return container_.template isSet<Ret>();
//}

#endif /* UAVAP_CORE_OBJECT_AGGREGATABLEOBJECT_HPP_ */
