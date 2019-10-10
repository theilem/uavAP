/*
 * ConfigurableObject.hpp
 *
 *  Created on: Jun 24, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_PROPERTYMAPPER_CONFIGURABLEOBJECT_HPP_
#define UAVAP_CORE_PROPERTYMAPPER_CONFIGURABLEOBJECT_HPP_
#include <uavAP/Core/PropertyMapper/Configuration.h>

template<class ParameterSet>
class ConfigurableObject
{
public:

	using ParamType = ParameterSet;

	ConfigurableObject() = default;

	ConfigurableObject(const ParameterSet& params);

	void
	setParams(const ParameterSet& set);

	bool
	configure(const Configuration& config);

	const ParameterSet&
	getParams() const;

	ParameterSet&
	getParams();

protected:

	template <typename Config>
	void
	configureParams(Config& config);

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
