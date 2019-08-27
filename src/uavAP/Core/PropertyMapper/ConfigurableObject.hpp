/*
 * ConfigurableObject.hpp
 *
 *  Created on: Jun 24, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_PROPERTYMAPPER_CONFIGURABLEOBJECT_HPP_
#define UAVAP_CORE_PROPERTYMAPPER_CONFIGURABLEOBJECT_HPP_
#include <uavAP/Core/PropertyMapper/Configuration.h>

template <class ParameterSet>
class ConfigurableObject
{
public:

	ConfigurableObject() = default;

	ConfigurableObject(const ParameterSet& params);

	bool
	configure(const Configuration& config);

	void
	setParams(const ParameterSet& set);

	std::string
	getJson();

protected:

	ParameterSet params;
};

#endif /* UAVAP_CORE_PROPERTYMAPPER_CONFIGURABLEOBJECT_HPP_ */
