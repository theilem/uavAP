/*
 * ConfigurableObjectImpl.hpp
 *
 *  Created on: Jun 24, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_PROPERTYMAPPER_CONFIGURABLEOBJECTIMPL_HPP_
#define UAVAP_CORE_PROPERTYMAPPER_CONFIGURABLEOBJECTIMPL_HPP_
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/PropertyMapper/ConfigurableObject.hpp"


template<class ParameterSet>
inline bool
ConfigurableObject<ParameterSet>::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);
	params.template configure(pm);
	return pm.map();
}


#endif /* UAVAP_CORE_PROPERTYMAPPER_CONFIGURABLEOBJECTIMPL_HPP_ */
