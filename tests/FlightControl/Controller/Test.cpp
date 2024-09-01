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
 * Test.cpp
 *
 *  Created on: Jun 30, 2017
 *      Author: mircot
 */

#include <boost/test/unit_test.hpp>

class TestingChannelMixing: public IChannelMixing, public IAggregatableObject
{
public:
	void
	setControllerOutput(const ControllerOutput& control) override
	{
		controlLog = control;
	}

	void
	notifyAggregationOnUpdate(const Aggregator& agg)
	{
	}

	ControllerOutput controlLog;
};

BOOST_AUTO_TEST_SUITE(GeneralTesting)

BOOST_AUTO_TEST_CASE(test1)
{
	auto gp = std::make_shared<FilletGlobalPlanner>();
	auto lp = std::make_shared<LinearLocalPlanner>();
	auto controller = std::make_shared<PIDController>();
	auto channelMix = std::make_shared<TestingChannelMixing>();
	auto scheduler = std::make_shared<MicroSimulator>();
	auto data = std::make_shared<FlightControlData>();

	Aggregator agg;
	agg.add(gp);
	agg.add(lp);
	agg.add(controller);
	agg.add(channelMix);
	agg.add(scheduler);
	agg.add(data);
}

BOOST_AUTO_TEST_SUITE_END()
