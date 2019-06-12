////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/*
 * AggregatableObject.h
 *
 *  Created on: Jun 29, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_OBJECT_IAGGREGATABLEOBJECT_H_
#define UAVAP_CORE_OBJECT_IAGGREGATABLEOBJECT_H_

#include "uavAP/Core/PropertyMapper/Configuration.h"

class Aggregator;

class IAggregatableObject
{

public:

	virtual
	~IAggregatableObject()
	{
	}


	using TypeId = const char* const;

	virtual void
	notifyAggregationOnUpdate(const Aggregator& agg) = 0;
};

#define ADD_CREATE_WITH_CONFIG(obj) \
static inline std::shared_ptr<obj> \
create(const Configuration& config) \
{\
	auto agg = std::make_shared<obj>();\
	if (!agg->configure(config))\
	{\
		APLOG_ERROR << #obj << ": Configuration failed";\
	}\
	return agg;\
}\

#define ADD_CREATE_WITHOUT_CONFIG(obj) \
static inline std::shared_ptr<obj> \
create(const Configuration& config) \
{\
	return std::make_shared<obj>();\
}

#endif /* UAVAP_CORE_OBJECT_IAGGREGATABLEOBJECT_H_ */
