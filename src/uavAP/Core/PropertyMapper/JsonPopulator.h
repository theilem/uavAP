/*
 * JsonPopulator.h
 *
 *  Created on: Jul 23, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_PROPERTYMAPPER_JSONPOPULATOR_H_
#define UAVAP_CORE_PROPERTYMAPPER_JSONPOPULATOR_H_
#include <uavAP/Core/EnumMap.hpp>
#include <uavAP/Core/Framework/Helper.h>
#include <uavAP/Core/PropertyMapper/ConfigurableObject.hpp>
#include <uavAP/Core/PropertyMapper/Parameter.h>
#include <uavAP/Core/TypeTraits.hpp>
#include <sstream>
#include <string>
#include <iostream>

class JsonPopulator
{
public:

	JsonPopulator();

	JsonPopulator(std::ofstream& file);

	~JsonPopulator();

	template<typename Type>
	typename std::enable_if<
			!is_parameter_set<typename Type::ValueType>::value
					&& !is_configurable_object<typename Type::ValueType>::value, JsonPopulator>::type&
	operator&(Type& param);

	template<typename Type>
	typename std::enable_if<(is_parameter_set<typename Type::ValueType>::value), JsonPopulator>::type&
	operator&(Type& param);

	template<typename Type>
	typename std::enable_if<(is_configurable_object<typename Type::ValueType>::value), JsonPopulator>::type&
	operator&(Type& param);

	std::string
	getString() const;

	void
	addTabs();

	void
	indent();

	void
	outdent();

	template<typename Type>
	JsonPopulator&
	operator <<(const Type& text);

	template<class Obj, typename std::enable_if<is_configurable_object<Obj>::value, int>::type = 0>
	void
	populate();

	template<class Obj, typename std::enable_if<!is_configurable_object<Obj>::value, int>::type = 0>
	void
	populate();

	template<class Obj1, class Obj2, class ... Others>
	void
	populate();

private:

	template<typename Type>
	typename std::enable_if<is_string<Type>::value, JsonPopulator>::type&
	writeValue(const Type& value);

	template<typename Type>
	typename std::enable_if<std::is_enum<Type>::value, JsonPopulator>::type&
	writeValue(const Type& value);

	template<typename Type>
	typename std::enable_if<!std::is_enum<Type>::value && !is_string<Type>::value, JsonPopulator>::type&
	writeValue(const Type& value);

	std::stringstream stringStream_;
	std::ostream jsonString_;

	bool firstElement_ = true;
	int tabCounter_ = 0;
};

template<typename Type>
inline typename std::enable_if<
!is_parameter_set<typename Type::ValueType>::value
		&& !is_configurable_object<typename Type::ValueType>::value, JsonPopulator>::type&
JsonPopulator::operator &(Type& param)
{
	if (!firstElement_)
	{
		jsonString_ << "," << std::endl;
	}
	addTabs();

	firstElement_ = false;
	jsonString_ << "\"" << param.id << "\":";
	this->template writeValue<typename Type::ValueType>(param.value);

	return *this;
}

template<typename Type>
inline typename std::enable_if<(is_parameter_set<typename Type::ValueType>::value), JsonPopulator>::type&
JsonPopulator::operator &(Type& param)
{
	if (!firstElement_)
	{
		jsonString_ << "," << std::endl;
	}
	addTabs();

	jsonString_ << "\"" << param.id << "\":{" << std::endl;

	firstElement_ = true;
	tabCounter_++;
	param.value.configure(*this);
	firstElement_ = false;
	tabCounter_--;

	jsonString_ << std::endl;
	addTabs();
	jsonString_ << "}";
	return *this;
}

template<typename Type>
inline typename std::enable_if<(is_configurable_object<typename Type::ValueType>::value), JsonPopulator>::type&
JsonPopulator::operator &(Type& param)
{
	if (!firstElement_)
	{
		jsonString_ << "," << std::endl;
	}
	addTabs();

	jsonString_ << "\"" << param.id << "\":{" << std::endl;

	firstElement_ = true;
	tabCounter_++;
	param.value.configureParams(*this);
	firstElement_ = false;
	tabCounter_--;

	jsonString_ << std::endl;
	addTabs();
	jsonString_ << "}";
	return *this;
}

template<typename Type>
inline typename std::enable_if<is_string<Type>::value, JsonPopulator>::type&
JsonPopulator::writeValue(const Type& value)
{
	jsonString_ << "\"" << value << "\"";
	return *this;
}

template<typename Type>
inline typename std::enable_if<std::is_enum<Type>::value, JsonPopulator>::type&
JsonPopulator::writeValue(const Type& value)
{
	jsonString_ << EnumMap<Type>::convert(value);
	return *this;
}

template<typename Type>
inline typename std::enable_if<!std::is_enum<Type>::value && !is_string<Type>::value, JsonPopulator>::type&
JsonPopulator::writeValue(const Type& value)
{
	jsonString_ << value;
	return *this;
}

template<typename Type>
inline JsonPopulator&
JsonPopulator::operator <<(const Type& text)
{
	jsonString_ << text;
	return *this;
}

template<class Obj, typename std::enable_if<is_configurable_object<Obj>::value, int>::type>
inline void
JsonPopulator::populate()
{
	//Is configurable Object
	if (!firstElement_)
	{
		jsonString_ << "," << std::endl;
	}
	addTabs();
	jsonString_ << "\"" << Obj::typeId << "\"" << ":{" << std::endl;
//	addTabs();
	indent();
	Obj p;
	firstElement_ = true;
	p.configureParams(*this);
	outdent();
	firstElement_ = false;

	jsonString_ << std::endl;
	addTabs();
	jsonString_ << "}";
}

template<class Obj, typename std::enable_if<!is_configurable_object<Obj>::value, int>::type>
inline void
JsonPopulator::populate()
{
	//Is not configurable, just add type
	if (!firstElement_)
	{
		jsonString_ << "," << std::endl;
	}
	addTabs();
	firstElement_ = false;
	jsonString_ << "\"" << Obj::typeId << "\"" << ":{" << std::endl;
	addTabs();
	jsonString_ << "}";
}

template<class Obj1, class Obj2, class ... Others>
inline void
JsonPopulator::populate()
{
	populate<Obj1>();
	populate<Obj2, Others...>();
}

#endif /* UAVAP_CORE_PROPERTYMAPPER_JSONPOPULATOR_H_ */
