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

	bool
	configure(const Configuration& config);

protected:

	ParameterSet params;
};



#endif /* UAVAP_CORE_PROPERTYMAPPER_CONFIGURABLEOBJECT_HPP_ */
