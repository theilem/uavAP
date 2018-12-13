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

#include <boost/signals2.hpp>
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Runner/IRunnableObject.h"

#include <memory>
#include <vector>

void
sigIntHandler(int sig) __attribute__((noreturn));

class Aggregator
{
public:

	Aggregator();

	template<class Type>
	void
	add(std::shared_ptr<Type> obj);

	void
	add(std::shared_ptr<IAggregatableObject> obj);

	void
	add(std::vector<std::shared_ptr<IAggregatableObject> > obj);

	template<class Type>
	std::shared_ptr<Type>
	getOne(Type* self = nullptr) const;

	template<class Type>
	std::vector<std::shared_ptr<Type> >
	getAll(Type* self = nullptr) const;

	static Aggregator
	aggregate(std::vector<std::shared_ptr<IAggregatableObject> > aggregation);

	using OnSIGINT = boost::signals2::signal<void(int)>;

	void
	subscribeOnSigint(const OnSIGINT::slot_type& slot) const;

	void
	callSigHandlers(int sig);

	void
	merge(Aggregator& agg);

	void
	mergeInto(Aggregator& agg);

private:

	std::vector<std::shared_ptr<IAggregatableObject> > container_;

	mutable OnSIGINT onSigint_;

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
	for (auto it : container_)
	{
		if (auto ret = std::dynamic_pointer_cast<Type>(it))
		{
			if (ret.get() == self)
				continue;
			return ret;
		}
	}
	return nullptr;
}

template<class Type>
inline std::vector<std::shared_ptr<Type> >
Aggregator::getAll(Type* self) const
{
	std::vector<std::shared_ptr<Type> > vec;
	for (auto it : container_)
	{
		if (auto ret = std::dynamic_pointer_cast<Type>(it))
		{
			if (ret.get() == self)
				continue;
			vec.push_back(ret);
		}
	}
	return vec;
}

#endif /* UAVAP_CORE_OBJECT_AGGREGATOR_H_ */
