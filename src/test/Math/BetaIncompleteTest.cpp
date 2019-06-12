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
 * BetaIncompleteTest.cpp
 *
 *  Created on: Aug 14, 2018
 *      Author: mircot
 */

#include <boost/test/unit_test.hpp>
#include <acb_hypgeom.h>
#include <boost/thread/thread_time.hpp>
#include <uavAP/Core/Time.h>
#include <iostream>

BOOST_AUTO_TEST_SUITE(BetaIncompleteTest)

BOOST_AUTO_TEST_CASE(test001_single)
{
	double g = 9.81;
	double v = 15;
	double r = 5.0 / 180.0 * M_PI;
	double phi = 30.0 / 180.0 * M_PI;

	acb_t a, b, z, z1, res1, res, pos;
	acb_init(a);
	acb_init(b);
	acb_init(z);
	acb_init(z1);
	acb_init(res);
	acb_init(res1);
	acb_init(pos);

	arb_t f;
	arb_init(f);

	arb_set_d(f, v / (2 * r));

	acb_set_d(z, pow(cos(phi), 2));
	acb_one(z1);

	acb_set_d_d(a, 1.0 / 2.0, -g / (2 * v * r));
	acb_set_d(b, 1.0 / 2.0);

	TimePoint t = Clock::now();
	acb_hypgeom_beta_lower(res1, a, b, z1, 0, 32);
	acb_hypgeom_beta_lower(res, a, b, z, 0, 32);
	acb_sub(res, res1, res, 32);
	acb_mul_arb(pos, res, f, 32);
	Duration d = Clock::now() - t;

//	std::cout << "Time: " << d.total_microseconds() << " micros" << std::endl;
//
//	std::cout << std::endl;
//	acb_printd(res, 6);
//	std::cout << std::endl;
//	acb_printd(pos, 6);
//	std::cout << std::endl;
//	std::cout << std::endl;

	acb_clear(a);
	acb_clear(b);
	acb_clear(z);
	acb_clear(z1);
	acb_clear(res);
	acb_clear(res1);
	acb_clear(pos);

	arb_clear(f);
}

BOOST_AUTO_TEST_CASE(test002_multi)
{
	double g = 9.81;
	double v = 15;
	double r = 15.0 / 180.0 * M_PI;
	double phi0 = 0.0 / 180.0 * M_PI;
	double phi1 = 10.0 / 180.0 * M_PI;
	double phi2 = 15.0 / 180.0 * M_PI;
	double phi3 = 20.0 / 180.0 * M_PI;
	double phi4 = 30.0 / 180.0 * M_PI;

	acb_t a, b;
	acb_ptr res, pos, z;
	acb_init(a);
	acb_init(b);

	arb_t f;
	arb_init(f);

	res = _acb_vec_init(5);
	z = _acb_vec_init(5);
	pos = _acb_vec_init(4);

	acb_set_d(z, pow(cos(phi0), 2));
	acb_set_d(z + 1, pow(cos(phi1), 2));
	acb_set_d(z + 2, pow(cos(phi2), 2));
	acb_set_d(z + 3, pow(cos(phi3), 2));
	acb_set_d(z + 4, pow(cos(phi4), 2));

	acb_set_d_d(a, 1.0 / 2.0, -g / (2 * v * r));
	acb_set_d(b, 1.0 / 2.0);

	arb_set_d(f, v / (2 * r));

	slong prec = 32;

	TimePoint t = Clock::now();
	_acb_hypgeom_beta_lower_series(res, a, b, z, 5, 0, 5, prec);
	acb_sub(res + 1, res, res + 1, prec);
	acb_sub(res + 2, res, res + 2, prec);
	acb_sub(res + 3, res, res + 3, prec);
	acb_sub(res + 4, res, res + 4, prec);
	acb_mul_arb(pos, res + 1, f, prec);
	acb_mul_arb(pos + 1, res + 2, f, prec);
	acb_mul_arb(pos + 2, res + 3, f, prec);
	acb_mul_arb(pos + 3, res + 4, f, prec);
	Duration d = Clock::now() - t;
//
//	std::cout << "Time: " << d.total_microseconds() << " micros" << std::endl;
//
//	acb_printd(res, 6);
//	std::cout << std::endl;
//	acb_printd(pos, 6);
//	std::cout << std::endl;
//	acb_printd(pos + 1, 6);
//	std::cout << std::endl;
//	acb_printd(pos + 2, 6);
//	std::cout << std::endl;
//	acb_printd(pos + 3, 6);
//	std::cout << std::endl;

	acb_clear(a);
	acb_clear(b);
	_acb_vec_clear(z, 5);
	_acb_vec_clear(res, 5);

}

BOOST_AUTO_TEST_SUITE_END()
