/*
 * AggregatableObjectImpl.hpp
 *
 *  Created on: Jun 12, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_OBJECT_AGGREGATABLEOBJECTIMPL_HPP_
#define UAVAP_CORE_OBJECT_AGGREGATABLEOBJECTIMPL_HPP_
#include <uavAP/Core/Object/AggregatableObject.hpp>


template<class ... Objects>
inline void
AggregatableObject<Objects...>::notifyAggregationOnUpdate(const Aggregator& agg)
{
	container_.setFromAggregationIfNotSet(agg);
}

template<class ... Objects>
template <class Agg>
void
AggregatableObject<Objects...>::notifyAggregationOnUpdate(const Agg& agg)
{
	container_.setFromAggregationIfNotSet(agg);
}

template<class ... Objects>
template<class Ret>
inline AggregatableObject<Objects...>::PtrType<Ret>
AggregatableObject<Objects...>::get()
{
	return container_.template get<Ret>();
}

template<class ... Objects>
template<class Ret>
inline bool
AggregatableObject<Objects...>::isSet() const
{
	return container_.template isSet<Ret>();
}

template<class ... Objects>
template<class Check>
inline bool
AggregatableObject<Objects...>::checkIsSet() const
{
	return container_.template isSet<Check>();
}

template<class ... Objects>
template<class Check, class ... Others>
inline bool
AggregatableObject<Objects...>::checkIsSet() const
{
	return container_.template isSet<Check>() && checkIsSet<Others...>;
}


#endif /* UAVAP_CORE_OBJECT_AGGREGATABLEOBJECTIMPL_HPP_ */
