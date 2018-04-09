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
 * ObjectHandle.h
 *
 *  Created on: Jun 29, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_OBJECT_OBJECTHANDLE_H_
#define UAVAP_CORE_OBJECT_OBJECTHANDLE_H_

#include "uavAP/Core/Object/Aggregator.h"

#include <memory>

template<class Handle>
class ObjectHandle
{
public:

	inline void
	setFromAggregationIfNotSet(const Aggregator& agg)
	{
		if (handle_.lock())
			return; //Already set

		handle_ = agg.getOne<Handle>();
	}

	inline std::shared_ptr<Handle>
	get() const
	{
		return handle_.lock();
	}

	inline bool
	isSet() const
	{
		return handle_.lock() != nullptr;
	}

	inline void
	set(std::weak_ptr<Handle> handle)
	{
		handle_ = handle;
	}

private:

	std::weak_ptr<Handle> handle_;
};

#endif /* UAVAP_CORE_OBJECT_OBJECTHANDLE_H_ */
