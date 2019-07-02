/*
 * Configuration.h
 *
 *  Created on: Jun 12, 2019
 *      Author: mirco
 */

#ifndef UAVAP_UAVAP_CORE_PROPERTYMAPPER_CONFIGURATION_H_
#define UAVAP_UAVAP_CORE_PROPERTYMAPPER_CONFIGURATION_H_

#ifdef ERIKA

#include "uavAP/Core/PropertyMapper/EmptyConfiguration.h"

using Configuration = EmptyConfiguration;

#else

#include <boost/property_tree/ptree.hpp>

using Configuration = boost::property_tree::ptree;

#endif



#endif /* UAVAP_UAVAP_CORE_PROPERTYMAPPER_CONFIGURATION_H_ */
