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
 * APLoggerTest.cpp
 *
 *  Created on: Jun 24, 2017
 *      Author: mircot
 */

#include <boost/test/unit_test.hpp>

#include "uavAP/Core/Object/AggregatableObject.hpp"
#include "uavAP/Core/Object/AggregatableObjectImpl.hpp"

namespace
{

class TestA;
class TestB;
class TestC;

class TestA : public AggregatableObject<TestB, TestC>
{
public:

	int testVal;


};

class TestB : public AggregatableObject<TestA, TestC>
{
public:

	int testVal;

};

class TestC : public AggregatableObject<TestA, TestB>
{
public:

	int testVal;



};

}

BOOST_AUTO_TEST_SUITE(ObjectHandleTests)

BOOST_AUTO_TEST_CASE(Aggregation)
{
	auto testa = std::make_shared<TestA>();
	auto testb = std::make_shared<TestB>();
	auto testc = std::make_shared<TestC>();

	testa->testVal = 1;

	BOOST_CHECK(!testa->isSet<TestB>());

	auto agg = Aggregator::aggregate({testa, testb, testc});

	BOOST_REQUIRE(testa->isSet<TestB>());

	BOOST_CHECK_EQUAL(testb->get<TestA>()->testVal, 1);
	BOOST_CHECK_EQUAL(testc->get<TestA>()->testVal, 1);

	testb->get<TestA>()->testVal = 2;

	BOOST_CHECK_EQUAL(testa->testVal, 2);



}



BOOST_AUTO_TEST_SUITE_END()






























