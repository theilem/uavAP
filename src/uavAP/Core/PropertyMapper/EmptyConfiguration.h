/*
 * EmptyConfiguration.h
 *
 *  Created on: Jun 11, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_PROPERTYMAPPER_EMPTYCONFIGURATION_H_
#define UAVAP_CORE_PROPERTYMAPPER_EMPTYCONFIGURATION_H_
#include <uavAP/Core/PropertyMapper/Optional.hpp>
#include <map>
#include <vector>

class EmptyConfiguration
{
public:
	template<typename Type>
	Type
	get(const std::string&) const;

	template<typename Type>
	Optional<Type>
	get_optional(const std::string&) const;

	Optional<EmptyConfiguration>
	get_child_optional(const std::string&) const;

	std::map<std::string, EmptyConfiguration> emptyVector = std::map<std::string, EmptyConfiguration>();

	std::map<std::string, EmptyConfiguration>::iterator
	begin();

	std::map<std::string, EmptyConfiguration>::iterator
	end();
};

template<typename Type>
inline Type
EmptyConfiguration::get(const std::string&) const
{
	return Type();
}

template<typename Type>
inline Optional<Type>
EmptyConfiguration::get_optional(const std::string&) const
{
	return Optional<Type>();
}

inline Optional<EmptyConfiguration>
EmptyConfiguration::get_child_optional(const std::string& allocator) const
{
	return Optional<EmptyConfiguration>();
}

inline std::map<std::string, EmptyConfiguration>::iterator
EmptyConfiguration::begin()
{
	return emptyVector.begin();
}

inline std::map<std::string, EmptyConfiguration>::iterator
EmptyConfiguration::end()
{
	return emptyVector.end();
}

#endif /* UAVAP_CORE_PROPERTYMAPPER_EMPTYCONFIGURATION_H_ */
