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
 * PropertyMapperTest.cpp
 *
 *  Created on: Aug 12, 2018
 *      Author: mircot
 */

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/test/unit_test.hpp>
#include <uavAP/Core/PropertyMapper/Configuration.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>


BOOST_AUTO_TEST_SUITE(PropertyMapperTest)


BOOST_AUTO_TEST_CASE(vector_of_double)
{
	Configuration config;
	boost::property_tree::read_json("Core/config/pm_test.json",
			config);

	PropertyMapper<Configuration> pm(config);
	std::vector<double> vec;
	BOOST_REQUIRE(pm.addVector<std::vector<double>>("vec_double", vec, true));

	BOOST_REQUIRE_EQUAL(vec.size(), 4);
	BOOST_CHECK_EQUAL(vec[0], 1.0);
	BOOST_CHECK_EQUAL(vec[1], 2.0);
	BOOST_CHECK_EQUAL(vec[2], 3.0);
	BOOST_CHECK_EQUAL(vec[3], 4.0);

}

BOOST_AUTO_TEST_CASE(vector_of_vector2)
{
	Configuration config;
	boost::property_tree::read_json("Core/config/pm_test.json",
			config);

	PropertyMapper<Configuration> pm(config);
	std::vector<Configuration> edges;
	pm.addVector("edges", edges, true);

	std::vector<Vector2> vec;

	for (const auto& it : edges)
	{
		PropertyMapper<Configuration> edge(it);
		Vector2 e;
		if (edge.add("",e,true))
			vec.push_back(e);
	}

	BOOST_REQUIRE_EQUAL(vec.size(), 4);
	BOOST_CHECK_EQUAL(vec[0], Vector2(1,1));
	BOOST_CHECK_EQUAL(vec[1], Vector2(2,1));
	BOOST_CHECK_EQUAL(vec[2], Vector2(1,2));
	BOOST_CHECK_EQUAL(vec[3], Vector2(2,2));

}


BOOST_AUTO_TEST_SUITE_END()
