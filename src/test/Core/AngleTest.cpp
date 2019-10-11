/*
 * AngleTest.cpp
 *
 *  Created on: Oct 11, 2019
 *      Author: mirco
 */
#include <boost/test/unit_test.hpp>
#include <uavAP/Core/Angle.h>
#include <cmath>





BOOST_AUTO_TEST_SUITE(AngleTest)

BOOST_AUTO_TEST_CASE(Test1)
{

	Angle<float> a = 180.0;
	BOOST_CHECK_CLOSE(a(), M_PI, 1e-4);

	a = 90;
	BOOST_CHECK_CLOSE(a(), M_PI_2, 1e-4);


}


BOOST_AUTO_TEST_SUITE_END()
