/*
 * Configuration.h
 *
 *  Created on: Jun 12, 2019
 *      Author: mirco
 */

#ifndef UAVAP_UAVAP_CORE_PROPERTYMAPPER_CONFIGURATION_H_
#define UAVAP_UAVAP_CORE_PROPERTYMAPPER_CONFIGURATION_H_

#ifdef NO_RTTI

#include "uavAP/Core/PropertyMapper/EmptyConfiguration.h"

using Configuration = EmptyConfiguration;
using ConfigurationError = int;

#else

#include <boost/property_tree/ptree.hpp>

using Configuration = boost::property_tree::ptree;
using ConfigurationError = boost::property_tree::ptree_error;

#endif



#endif /* UAVAP_UAVAP_CORE_PROPERTYMAPPER_CONFIGURATION_H_ */
