/*
 * Parameter.h
 *
 *  Created on: Jun 24, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_PROPERTYMAPPER_PARAMETER_H_
#define UAVAP_CORE_PROPERTYMAPPER_PARAMETER_H_
#include <string>

template<typename Type>
struct Parameter
{
	using ValueType = Type;

	Parameter<Type>&
	operator=(const Parameter<Type>& other)
	{
		value = other.value;
		return *this;
	}

	Parameter<Type>&
	operator=(const Type& val)
	{
		value = val;
		return *this;
	}

	inline const Type&
	operator()() const
	{
		return value;
	}

	inline void
	setValue(const Type& val)
	{
		value = val;
	}

	Type value;
	const std::string id;
	const bool mandatory;

};

template<typename Type>
struct is_parameter_set
{
	template<typename _1>
	static char &
	chk(
			typename std::enable_if<
					std::is_same<void, decltype(std::declval<_1>().configure(std::declval<int&>()))>::value,
					int>::type);

	template<typename >
	static int &
	chk(...);

	static constexpr bool value = sizeof(chk<Type>(0)) == sizeof(char);
};

template<typename Type>
struct is_parameter: public std::false_type
{
};

template<typename Type>
struct is_parameter<Parameter<Type>> : public std::true_type
{
};

template<typename Type>
using enable_if_is_parameter = typename std::enable_if<is_parameter<Type>::value, Type>::type;
template<typename Type>
using enable_if_not_is_parameter = typename std::enable_if<!is_parameter<Type>::value, Type>::type;

template<typename Type>
using enable_if_is_parameter_set = typename std::enable_if<is_parameter_set<Type>::value, Type>::type;
template<typename Type>
using enable_if_not_is_parameter_set = typename std::enable_if<!(is_parameter_set<Type>::value), Type>::type;

#endif /* UAVAP_CORE_PROPERTYMAPPER_PARAMETER_H_ */
