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
 * SystemTimeProvider.h
 *
 *  Created on: Jul 19, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_TIMEPROVIDER_SYSTEMTIMEPROVIDER_H_
#define UAVAP_CORE_TIMEPROVIDER_SYSTEMTIMEPROVIDER_H_
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/TimeProvider/ITimeProvider.h"
#include "uavAP/Core/PropertyMapper/Configuration.h"

class SystemTimeProvider: public ITimeProvider, public IAggregatableObject
{
public:

	static constexpr TypeId typeId = "system";

	SystemTimeProvider();

	static std::shared_ptr<ITimeProvider>
	create(const Configuration& conf);

	TimePoint
	now() override;

	bool
	waitFor(Duration duration, std::condition_variable& interrupt,
			std::unique_lock<std::mutex>& lock) override;

	bool
	waitUntil(TimePoint timePoint, std::condition_variable& interrupt,
			std::unique_lock<std::mutex>& lock) override;

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

private:

	TimePoint chronoEpoch_;

};

#endif /* UAVAP_CORE_TIMEPROVIDER_SYSTEMTIMEPROVIDER_H_ */
