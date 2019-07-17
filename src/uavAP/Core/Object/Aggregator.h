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
 * Aggregator.h
 *
 *  Created on: Jun 29, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_OBJECT_AGGREGATOR_H_
#define UAVAP_CORE_OBJECT_AGGREGATOR_H_

#include <uavAP/Core/Object/DynamicContainer/DynamicObjectContainer.h>
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Runner/IRunnableObject.h"

#include <memory>
#include <vector>

class Aggregator
{
public:

	using ObjectContainer = DynamicObjectContainer;

	Aggregator();

	template<class Type>
	void
	add(std::shared_ptr<Type> obj);

	void
	add(std::shared_ptr<IAggregatableObject> obj);

	void
	add(const ObjectContainer& obj);

	template<class Type>
	std::shared_ptr<Type>
	getOne(Type* self = nullptr) const;

	template<class Type>
	std::vector<std::shared_ptr<Type> >
	getAll(Type* self = nullptr) const;

	static Aggregator
	aggregate(std::vector<std::shared_ptr<IAggregatableObject> > aggregation);

	void
	merge(Aggregator& agg);

	void
	mergeInto(Aggregator& agg);

	/**
	 * @brief Clear the container. Will destroy all the objects.
	 */
	void
	clear();

private:

	ObjectContainer container_;
};

template<class Type>
inline void
Aggregator::add(std::shared_ptr<Type> obj)
{
	if (auto aggObj = std::dynamic_pointer_cast<IAggregatableObject>(obj))
	{
		add(aggObj);
	}
}

template<class Type>
inline std::shared_ptr<Type>
Aggregator::getOne(Type* self) const
{
	return container_.template getOne<Type>(self);
}

template<class Type>
inline std::vector<std::shared_ptr<Type> >
Aggregator::getAll(Type* self) const
{
	return container_.template getAll<Type>(self);
}

#endif /* UAVAP_CORE_OBJECT_AGGREGATOR_H_ */
