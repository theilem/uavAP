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
 * ITimeProvider.h
 *
 *  Created on: Jul 19, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_TIMEPROVIDER_ITIMEPROVIDER_H_
#define UAVAP_CORE_TIMEPROVIDER_ITIMEPROVIDER_H_
#include <boost/thread/lock_types.hpp>
#include <boost/thread/pthread/condition_variable.hpp>
#include "uavAP/Core/Time.h"

class ITimeProvider
{
public:

	static constexpr const char* const typeId = "time_provider";

	virtual
	~ITimeProvider() = default;

	virtual TimePoint
	now() = 0;

	virtual bool
	waitFor(Duration duration, boost::condition_variable& interrupt,
			boost::unique_lock<boost::mutex>& lock) = 0;

	virtual bool
	waitUntil(TimePoint timePoint, boost::condition_variable& interrupt,
			boost::unique_lock<boost::mutex>& lock) = 0;

private:

};

#endif /* UAVAP_CORE_TIMEPROVIDER_ITIMEPROVIDER_H_ */
