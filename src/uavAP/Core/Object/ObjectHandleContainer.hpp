/*
 * ObjectHandleContainer.hpp
 *
 *  Created on: Jun 7, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_OBJECT_OBJECTHANDLECONTAINER_HPP_
#define UAVAP_CORE_OBJECT_OBJECTHANDLECONTAINER_HPP_
#include <uavAP/Core/Object/Aggregator.h>
#include <memory>

template<class ... Objects>
class ObjectHandleContainer;

template<class Object, class ... Others>
class ObjectHandleContainer<Object, Others...>
{
public:

	template<class Type>
	using PtrType = std::shared_ptr<Type>;
	template<class Type>
	using WeakPtrType = std::weak_ptr<Type>;
	using AggregatorType = Aggregator;

	template<class Ret>
	PtrType<
			typename std::enable_if<
					(std::is_base_of<Ret, Object> { } || std::is_same<Ret, Object> { }), Ret>::type>
	get();

	template<class Ret>
	PtrType<
			typename std::enable_if<
					!(std::is_base_of<Ret, Object> { } || std::is_same<Ret, Object> { }), Ret>::type>
	get();

	template<class Ret>
	typename std::enable_if<(std::is_base_of<Ret, Object> { } || std::is_same<Ret, Object> { }),
			bool>::type
	isSet() const;

	template<class Ret>
	typename std::enable_if<!(std::is_base_of<Ret, Object> { } || std::is_same<Ret, Object> { }),
			bool>::type
	isSet() const;

	void
	setFromAggregationIfNotSet(const AggregatorType& agg);

private:

	ObjectHandleContainer<Others...> others_;
	WeakPtrType<Object> object_;
};

template<>
class ObjectHandleContainer<>
{
public:

	template<class Type>
	using PtrType = std::shared_ptr<Type>;
	template<class Type>
	using WeakPtrType = std::weak_ptr<Type>;
	using AggregatorType = Aggregator;

	template<class Ret>
	inline std::shared_ptr<Ret>
	get()
	{
		return nullptr;
//		static_assert(false, "Cannot get object");
	}

	template<class Ret>
	inline bool
	isSet() const
	{
		return false;
//		static_assert(false, "Cannot get object");
	}

	inline void
	setFromAggregationIfNotSet(const AggregatorType&)
	{
	}

};

template<class Object, class ... Others>
template<class Ret>
inline ObjectHandleContainer<Object, Others...>::PtrType<
		typename std::enable_if<(std::is_base_of<Ret, Object> { } || std::is_same<Ret, Object> { }),
				Ret>::type>
ObjectHandleContainer<Object, Others...>::get()
{
	return object_.lock();
}

template<class Object, class ... Others>
template<class Ret>
inline ObjectHandleContainer<Object, Others...>::PtrType<
		typename std::enable_if<
				!(std::is_base_of<Ret, Object> { } || std::is_same<Ret, Object> { }), Ret>::type>
ObjectHandleContainer<Object, Others...>::get()
{
	return others_.template get<Ret>();
}

template<class Object, class ... Others>
inline void
ObjectHandleContainer<Object, Others ...>::setFromAggregationIfNotSet(const AggregatorType& agg)
{
	if (!object_.lock())
		object_ = agg.template getOne<Object>();
	others_.setFromAggregationIfNotSet(agg);
}

template<class Object, class ... Others>
template<class Ret>
inline typename std::enable_if<(std::is_base_of<Ret, Object> { } || std::is_same<Ret, Object> { }),
		bool>::type
ObjectHandleContainer<Object, Others...>::isSet() const
{
	return object_.lock() != nullptr;
}

template<class Object, class ... Others>
template<class Ret>
inline typename std::enable_if<!(std::is_base_of<Ret, Object> { } || std::is_same<Ret, Object> { }),
		bool>::type
ObjectHandleContainer<Object, Others...>::isSet() const
{
	return others_.template isSet<Ret>();
}

#endif /* UAVAP_CORE_OBJECT_OBJECTHANDLECONTAINER_HPP_ */
