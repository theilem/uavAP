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
 * ApExtTest.cpp
 *
 *  Created on: Jul 13, 2018
 *      Author: sim
 */

#include <boost/test/unit_test.hpp>
#include <uavAP/API/ap_ext/ap_ext.h>
#include <uavAP/Core/IPC/IPC.h>
#include <uavAP/FlightControl/Controller/AdvancedControl.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>


BOOST_AUTO_TEST_SUITE(ApExtTest)


BOOST_AUTO_TEST_CASE(test001)
{
	IPC ipc;
	auto pub = ipc.publishOnSharedMemory<ControllerOutput>("actuation");
	auto pubAdv = ipc.publishOnSharedMemory<AdvancedControl>("advanced_control");

	setConfigPath("/usr/local/config/sailplane/alvolo.json");
	BOOST_REQUIRE_EQUAL(ap_ext_setup(), 0);
	boost::this_thread::sleep(Milliseconds(10));

	unsigned long channels[32] = {0};


	AdvancedControl adv;
	adv.throwsSelection = ThrowsControl::NORMAL;
	adv.specialSelection = SpecialControl::FLAP;
	adv.specialValue = 1;

	pubAdv.publish(adv);

	ControllerOutput out;
	out.rollOutput = 0;
	out.pitchOutput = 0;
	out.yawOutput = 0;
	out.throttleOutput = 0;

	pub.publish(out);
	boost::this_thread::sleep(Milliseconds(10));

	ap_ext_actuate(channels, 7);

//	for (int i = 0; i < 32; i++)
//	{
//		std::cout << channels[i] << ",";
//	}
//	std::cout << std::endl;


	BOOST_REQUIRE_EQUAL(ap_ext_teardown(), 0);
}

BOOST_AUTO_TEST_SUITE_END()
