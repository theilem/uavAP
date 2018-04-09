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
 * TestSender.cpp
 *
 *  Created on: Jun 30, 2017
 *      Author: mircot
 */

#include <unistd.h>
#include <iostream>
#include <boost/interprocess/ipc/message_queue.hpp>
#include "uavAP/Core/LinearAlgebra.h"

#include "TestStruct.h"

int
main(int argc, char** argv)
{

	std::cout << "Sender Start" << std::endl;

	boost::interprocess::message_queue message(boost::interprocess::open_only, "test_message");

	std::cout << "Sender opened message" << std::endl;

	TestStruct vec =
	{ 1, 2,
	{ 1, 2, 3 } };

	usleep(1e6);

	message.send(&vec, sizeof(Vector3), 1);
	return 0;

}

