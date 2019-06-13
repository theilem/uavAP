/*
 * StaticObjectContainer.h
 *
 *  Created on: Jun 7, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_OBJECT_STATICCONTAINER_STATICOBJECTCONTAINER_H_
#define UAVAP_CORE_OBJECT_STATICCONTAINER_STATICOBJECTCONTAINER_H_
#include <memory>
#include <type_traits>
#include <vector>

template<class ... Objects>
struct StaticObjectContainer;
template<class ... Objects>
class StaticAggregator;

template<>
struct StaticObjectContainer<>
{
	template<class Ret>
	std::shared_ptr<Ret>
	getOne() const
	{
		return nullptr;
	}

	template<class Ret>
	std::vector<std::shared_ptr<Ret>>
	getAll() const
	{
		return std::vector<std::shared_ptr<Ret>>();
	}

	template<class ...Objects>
	void
	notifyAggregationOnUpdate(const StaticAggregator<Objects...>& agg);
};

template<class Object, class ... Others>
struct StaticObjectContainer<Object, Others...>
{

	StaticObjectContainer();

	StaticObjectContainer<Others...> others;
	Object object;
	std::shared_ptr<Object> objectPtr;

	template<class Ret>
	std::shared_ptr<
			typename std::enable_if<
					(std::is_base_of<Ret, Object> { } || std::is_same<Ret, Object> { }), Ret>::type>
	getOne() const;

	template<class Ret>
	std::shared_ptr<
			typename std::enable_if<
					!(std::is_base_of<Ret, Object> { } || std::is_same<Ret, Object> { }), Ret>::type>
	getOne() const;

	template<class Ret>
	std::vector<
			std::shared_ptr<
					typename std::enable_if<
							(std::is_base_of<Ret, Object> { } || std::is_same<Ret, Object> { }), Ret>::type>>
	getAll() const;

	template<class Ret>
	std::vector<
			std::shared_ptr<
					typename std::enable_if<
							!(std::is_base_of<Ret, Object> { } || std::is_same<Ret, Object> { }),
							Ret>::type>>
	getAll() const;

	template<class ...Objects>
	void
	notifyAggregationOnUpdate(const StaticAggregator<Objects...>& agg);

};

#include <uavAP/Core/Object/StaticAggregator.h>

template<class Object, class ... Others>
inline
StaticObjectContainer<Object, Others ...>::StaticObjectContainer() :
		others(), object(), objectPtr(&object, [](Object*)
		{})
{
}

template<class Object, class ... Others>
template<class Ret>
inline std::shared_ptr<
		typename std::enable_if<(std::is_base_of<Ret, Object> { } || std::is_same<Ret, Object> { }),
				Ret>::type>
StaticObjectContainer<Object, Others...>::getOne() const
{
	return objectPtr;
}

template<class Object, class ... Others>
template<class Ret>
inline std::shared_ptr<
		typename std::enable_if<
				!(std::is_base_of<Ret, Object> { } || std::is_same<Ret, Object> { }), Ret>::type>
StaticObjectContainer<Object, Others...>::getOne() const
{
	return others.template getOne<Ret>();
}

template<class Object, class ... Others>
template<class Ret>
inline std::vector<
		std::shared_ptr<
				typename std::enable_if<
						(std::is_base_of<Ret, Object> { } || std::is_same<Ret, Object> { }), Ret>::type>>
StaticObjectContainer<Object, Others...>::getAll() const
{
	auto vec = others.template getAll<Ret>();
	vec.push_back(objectPtr);
	return vec;
}

template<class Object, class ... Others>
template<class Ret>
inline std::vector<
		std::shared_ptr<
				typename std::enable_if<
						!(std::is_base_of<Ret, Object> { } || std::is_same<Ret, Object> { }), Ret>::type>>
StaticObjectContainer<Object, Others...>::getAll() const
{
	return others.template getAll<Ret>();
}

template<class Object, class ... Others>
template<class ...Objects>
inline void
StaticObjectContainer<Object, Others ...>::notifyAggregationOnUpdate(
		const StaticAggregator<Objects...>& agg)
{
	object.notifyAggregationOnUpdate(agg);
	others.notifyAggregationOnUpdate(agg);
}

template<class ... Objects>
inline void
StaticObjectContainer<>::notifyAggregationOnUpdate(const StaticAggregator<Objects...>& agg)
{
}

#endif /* UAVAP_CORE_OBJECT_STATICCONTAINER_STATICOBJECTCONTAINER_H_ */
