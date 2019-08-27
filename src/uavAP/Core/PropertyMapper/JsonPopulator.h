/*
 * JsonPopulator.h
 *
 *  Created on: Jul 23, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_PROPERTYMAPPER_JSONPOPULATOR_H_
#define UAVAP_CORE_PROPERTYMAPPER_JSONPOPULATOR_H_
#include <uavAP/Core/PropertyMapper/Parameter.h>
#include <sstream>
#include <string>

class JsonPopulator
{
public:

	JsonPopulator() = default;

	template <typename Type>
	void
	operator&(const Parameter<typename std::enable_if<!is_parameter_set<Type>::value, Type>::type>& param);

	template <typename Type>
	void
	operator&(const Parameter<typename std::enable_if<is_parameter_set<Type>::value, Type>::type>& param);

	std::string
	getString() const;

private:

	std::stringstream jsonString_;
};

template<typename Type>
inline void
JsonPopulator::operator &(const Parameter<typename std::enable_if<!is_parameter_set<Type>::value, Type>::type>& param)
{
	jsonString_ << "\"" << param.id << "\" : " << param.value;
}

template<typename Type>
inline void
JsonPopulator::operator &(const Parameter<typename std::enable_if<is_parameter_set<Type>::value, Type>::type>& param)
{
//	jsonString_ << "\"" << param.id << "\" : " << param.value;
}

#endif /* UAVAP_CORE_PROPERTYMAPPER_JSONPOPULATOR_H_ */
