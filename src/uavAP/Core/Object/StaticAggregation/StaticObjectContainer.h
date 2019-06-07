/*
 * StaticObjectContainer.h
 *
 *  Created on: Jun 7, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_OBJECT_STATICAGGREGATION_STATICOBJECTCONTAINER_H_
#define UAVAP_CORE_OBJECT_STATICAGGREGATION_STATICOBJECTCONTAINER_H_
#include <type_traits>


template<class... Objects>
struct StaticObjectContainer;

template<>
struct StaticObjectContainer<>
{
	template <class Ret>
	Ret*
	get()
	{
		return nullptr;
	}
};


template<class Object, class... Others>
struct StaticObjectContainer<Object, Others...>
{
	StaticObjectContainer<Others...> others;
	Object object;

	template <class Ret>
	typename std::enable_if<(std::is_base_of<Ret, Object>{} || std::is_same<Ret,Object>{}),Ret>::type*
	get();

	template <class Ret>
	typename std::enable_if<!(std::is_base_of<Ret, Object>{} || std::is_same<Ret,Object>{}),Ret>::type*
	get();


};

template<class Object, class... Others>
template<class Ret>
inline typename std::enable_if<(std::is_base_of<Ret, Object>
		{}|| std::is_same<Ret,Object>
		{}),Ret>::type* StaticObjectContainer<Object, Others...>::get()
{
	return &object;
}

template<class Object, class... Others>
template<class Ret>
inline typename std::enable_if<!(std::is_base_of<Ret, Object>
		{}|| std::is_same<Ret,Object>
		{}),Ret>::type* StaticObjectContainer<Object, Others...>::get()
{
	return others.template get<Ret>();
}


#endif /* UAVAP_CORE_OBJECT_STATICAGGREGATION_STATICOBJECTCONTAINER_H_ */
