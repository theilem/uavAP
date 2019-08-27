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
 * LowPassDataFilter.h
 *
 *  Created on: Mar 12, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_DATAFILTER_LOWPASSDATAFILTER_LOWPASSDATAFILTER_H_
#define UAVAP_CORE_DATAPRESENTATION_DATAFILTER_LOWPASSDATAFILTER_LOWPASSDATAFILTER_H_

#include "uavAP/Core/LinearAlgebra.h"
#include "uavAP/Core/PropertyMapper/Configuration.h"
#include "uavAP/Core/DataPresentation/DataFilter/IDataFilter.h"

class LowPassDataFilter: public IDataFilter
{
public:

	static constexpr const char * const typeId = "low_pass";

	LowPassDataFilter();

	LowPassDataFilter(FloatingType initialValue, FloatingType alpha);

	static std::shared_ptr<LowPassDataFilter>
	create(const Configuration& config);

	bool
	configure(const Configuration& config);

	void
	initialize(FloatingType initialValue) override;

	void
	tune(FloatingType alpha) override;

	void
	filterData(FloatingType rawData) override;

	FloatingType
	getFilteredData() override;

private:

	FloatingType filteredData_;
	FloatingType alpha_;
};

#endif /* UAVAP_CORE_DATAPRESENTATION_DATAFILTER_LOWPASSDATAFILTER_LOWPASSDATAFILTER_H_ */
