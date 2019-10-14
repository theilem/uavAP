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
#include <iostream>

template<class ParameterSet>
ConfigurableObject<ParameterSet>::ConfigurableObject(const ParameterSet& p) :
		params(p)
{
}

template<class ParameterSet>
inline bool
ConfigurableObject<ParameterSet>::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);
	params.template configure(pm);
	return pm.map();
}

template<class ParameterSet>
inline void
ConfigurableObject<ParameterSet>::setParams(const ParameterSet& set)
{
	params = set;
}

template<class ParameterSet>
inline const ParameterSet&
ConfigurableObject<ParameterSet>::getParams() const
{
	return params;
}

template<class ParameterSet>
inline ParameterSet&
ConfigurableObject<ParameterSet>::getParams()
{
	return params;
}

template <typename ParameterSet>
template<typename Config>
inline void
ConfigurableObject<ParameterSet>::configureParams(Config& config)
{
	params.template configure(config);
}

#endif /* UAVAP_CORE_PROPERTYMAPPER_CONFIGURABLEOBJECTIMPL_HPP_ */
