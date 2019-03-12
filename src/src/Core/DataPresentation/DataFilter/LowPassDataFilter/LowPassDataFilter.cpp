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

#include "uavAP/Core/DataPresentation/DataFilter/LowPassDataFilter/LowPassDataFilter.h"

LowPassDataFilter::LowPassDataFilter(double initialValue, double alpha) :
		filteredData_(initialValue), alpha_(alpha)
{
}

void
LowPassDataFilter::initialize(double initialValue, double alpha)
{
	filteredData_ = initialValue;
	alpha_ = alpha;
}

void
LowPassDataFilter::filterData(double rawData)
{
	filteredData_ = alpha_ * rawData + (1 - alpha_) * filteredData_;
}

double
LowPassDataFilter::getFilteredData()
{
	return filteredData_;
}

void
LowPassDataFilter::setAlpha(double alpha)
{
	alpha_ = alpha;
}
