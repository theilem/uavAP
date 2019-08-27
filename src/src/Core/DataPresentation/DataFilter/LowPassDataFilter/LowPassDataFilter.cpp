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
 * LowPassDataFilter.cpp
 *
 *  Created on: Mar 12, 2019
 *      Author: simonyu
 */

#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/DataPresentation/DataFilter/LowPassDataFilter/LowPassDataFilter.h"

LowPassDataFilter::LowPassDataFilter() :
		filteredData_(0), alpha_(1.0)
{
}

LowPassDataFilter::LowPassDataFilter(FloatingType initialValue, FloatingType alpha) :
		filteredData_(initialValue), alpha_(alpha)
{
}

std::shared_ptr<LowPassDataFilter>
LowPassDataFilter::create(const Configuration& config)
{
	auto lowPassDataFilter = std::make_shared<LowPassDataFilter>();

	if (!lowPassDataFilter->configure(config))
	{
		APLOG_ERROR << "LowPassDataFilter: Failed to Load Config.";
	}

	return lowPassDataFilter;
}

bool
LowPassDataFilter::configure(const Configuration& config)
{
	PropertyMapper<Configuration> pm(config);

	pm.add<FloatingType>("alpha", alpha_, true);

	return pm.map();
}

void
LowPassDataFilter::initialize(FloatingType initialValue)
{
	filteredData_ = initialValue;
}

void
LowPassDataFilter::tune(FloatingType alpha)
{
	alpha_ = alpha;
}

void
LowPassDataFilter::filterData(FloatingType rawData)
{
	filteredData_ = alpha_ * rawData + (1 - alpha_) * filteredData_;
}

FloatingType
LowPassDataFilter::getFilteredData()
{
	return filteredData_;
}
