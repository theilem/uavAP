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
 * testman.cpp
 *
 *  Created on: Jul 27, 2018
 *      Author: mircot
 */
#include <boost/test/unit_test.hpp>

#include "uavAP/Core/EnumMap.hpp"

enum class ThisTest
{
	Test1 = 7, Test2, Test3
};

ENUMMAP_INIT(ThisTest, { {ThisTest::Test1, "test1"}, {ThisTest::Test2, "test2"} });

enum class ThisTest2
{
	Test1 = 9, Test2, Test3
};

ENUMMAP_INIT(ThisTest2, { {ThisTest2::Test1, "test122"}, {ThisTest2::Test2, "test2"} });

BOOST_AUTO_TEST_SUITE(ManualTests)

BOOST_AUTO_TEST_CASE(TestMan)
{
	std::cout << EnumMap<ThisTest>::convert(ThisTest::Test1) << std::endl;
	std::cout << static_cast<int>(EnumMap<ThisTest>::convert("test1")) << std::endl;
	std::cout << EnumMap<ThisTest2>::convert(ThisTest2::Test1) << std::endl;
	std::cout << static_cast<int>(EnumMap<ThisTest2>::convert("test2")) << std::endl;
}

BOOST_AUTO_TEST_SUITE_END()
