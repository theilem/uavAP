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
 * Aggregator.cpp
 *
 *  Created on: Aug 6, 2017
 *      Author: mircot
 */
#include "uavAP/Core/Object/Aggregator.h"
#include <csignal>

static Aggregator* aggSigHandler = nullptr;

void
sigIntHandler(int sig)
{
	if (aggSigHandler)
	{
		aggSigHandler->callSigHandlers(sig);
	}
	exit(sig);
}

Aggregator::Aggregator()
{
	if (!aggSigHandler)
	{
		aggSigHandler = this;
		std::signal(SIGINT, sigIntHandler);
		std::signal(SIGTERM, sigIntHandler);
	}
}

void
Aggregator::add(std::shared_ptr<IAggregatableObject> obj)
{
	container_.add(obj);
	container_.notifyAggregationOnUpdate(*this);
}

Aggregator
Aggregator::aggregate(std::vector<std::shared_ptr<IAggregatableObject> > aggregation)
{
	Aggregator agg;
	for (auto it : aggregation)
	{
		agg.add(it);
	}
	return agg;
}

void
Aggregator::add(const ObjectContainer& obj)
{
	container_.add(obj);
}

void
Aggregator::subscribeOnSigint(const OnSIGINT::slot_type& slot) const
{
	onSigint_.connect(slot);
}

void
Aggregator::callSigHandlers(int sig)
{
	onSigint_(sig);
}

void
Aggregator::merge(Aggregator& agg)
{
	agg.mergeInto(*this);
}

void
Aggregator::mergeInto(Aggregator& agg)
{
	agg.add(container_);
	container_.clear();
}

void
Aggregator::clear()
{
	container_.clear();
}
