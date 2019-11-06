/*
 * ConfigurableObject.hpp
 *
 *  Created on: Jun 24, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_PROPERTYMAPPER_CONFIGURABLEOBJECT_HPP_
#define UAVAP_CORE_PROPERTYMAPPER_CONFIGURABLEOBJECT_HPP_
#include "uavAP/Core/PropertyMapper/Configuration.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"

template<class ParameterSet>
class ConfigurableObject
{
public:

	using ParamType = ParameterSet;

	ConfigurableObject() = default;

	inline ConfigurableObject(const ParameterSet& p) :
			params(p)
	{
	}

	inline void
	setParams(const ParameterSet& set)
	{
		params = set;
	}

	inline bool
	configure(const Configuration& config)
	{
		PropertyMapper<Configuration> pm(config);
		params.template configure(pm);
		return pm.map();
	}

	inline const ParameterSet&
	getParams() const
	{
		return params;
	}

	inline ParameterSet&
	getParams()
	{
		return params;
	}

	template<typename Config>
	inline void
	configureParams(Config& config)
	{
		params.template configure(config);
	}

protected:

	ParameterSet params;
};

template<typename Type>
struct is_configurable_object
{
	template<typename _1>
	static char &
	chk(
			typename std::enable_if<
					std::is_same<void,
							decltype(std::declval<typename _1::ParamType>().configure(std::declval<int&>()))>::value,
					int>::type);

	template<typename >
	static int &
	chk(...);

	static constexpr bool value = sizeof(chk<Type>(0)) == sizeof(char);
};

#endif /* UAVAP_CORE_PROPERTYMAPPER_CONFIGURABLEOBJECT_HPP_ */
