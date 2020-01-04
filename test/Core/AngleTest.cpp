/*
 * AngleTest.cpp
 *
 *  Created on: Oct 11, 2019
 *      Author: mirco
 */
#include <boost/test/unit_test.hpp>
#include <uavAP/Core/Angle.h>
#include <uavAP/FlightControl/Controller/ControlElements/ControlElements.h>
#include <cmath>





BOOST_AUTO_TEST_SUITE(AngleTest)

BOOST_AUTO_TEST_CASE(Test1)
{

	Angle<float> a(180.0);
	BOOST_CHECK_CLOSE(a(), M_PI, 1e-4);

	a = 90;
	BOOST_CHECK_CLOSE(a(), M_PI_2, 1e-4);

	float rad = a;
	BOOST_CHECK_CLOSE(rad, M_PI_2, 1e-4);

	float deg = a.degrees();
	BOOST_CHECK_CLOSE(deg, 90.0, 1e-4);

}

BOOST_AUTO_TEST_CASE(AngleConstraint)
{
	double test = 0;
	double min = -90;
	double max = 90;

	auto in = std::make_shared<Control::Input>(&test);
	auto constraint = Control::Constraint<Angle<double>>(in, Angle<double>(min), Angle<double>(max));

	BOOST_CHECK_EQUAL(constraint.getValue(), test);

	test = 1;
	BOOST_CHECK_EQUAL(constraint.getValue(), test);

	test = 2;
	BOOST_CHECK_EQUAL(constraint.getValue(), M_PI_2);

	test = -2;
	BOOST_CHECK_EQUAL(constraint.getValue(), -M_PI_2);


	Configuration config;
	config.add("min", min);
	config.add("max", max);

	auto constraint2 = Control::Constraint<Angle<double>>(in);
	constraint2.configure(config);

	test = 0;
	BOOST_CHECK_EQUAL(constraint2.getValue(), test);

	test = 1;
	BOOST_CHECK_EQUAL(constraint2.getValue(), test);

	test = 2;
	BOOST_CHECK_EQUAL(constraint2.getValue(), M_PI_2);

	test = -2;
	BOOST_CHECK_EQUAL(constraint2.getValue(), -M_PI_2);

}

BOOST_AUTO_TEST_SUITE_END()
