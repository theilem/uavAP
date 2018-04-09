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
 * AggregatableRunner.cpp
 *
 *  Created on: Jul 1, 2017
 *      Author: mircot
 */

#include "uavAP/Core/Object/Aggregator.h"
#include "uavAP/Core/Runner/AggregatableRunner.h"

AggregatableRunner::AggregatableRunner()
{
}

bool
AggregatableRunner::runStage(RunStage stage)
{
	for (auto it : runnableObjects_)
	{
		if (it->run(stage))
			return true;
	}
	return false;
}

bool
AggregatableRunner::runAllStages()
{
	if (runStage(RunStage::INIT))
		return true;
	if (runStage(RunStage::NORMAL))
		return true;
	if (runStage(RunStage::FINAL))
		return true;
	return false;
}

void
AggregatableRunner::notifyAggregationOnUpdate(Aggregator& agg)
{
	runnableObjects_ = agg.getAll<IRunnableObject>();
}
