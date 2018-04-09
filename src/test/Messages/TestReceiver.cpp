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
 * TestReceiver.cpp
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

	std::cout << "Receiver Start" << std::endl;

	boost::interprocess::message_queue message(boost::interprocess::open_only, "test_message");

	std::cout << "Receiver opened message" << std::endl;

	boost::interprocess::message_queue::size_type receivedSize;
	unsigned int priority;
	size_t max = 1000;
	TestStruct receiveStruct;
	message.receive(&receiveStruct, max, receivedSize, priority);

	std::cout << receiveStruct.test2.x() << " " << receiveStruct.test2.y() << " "
			<< receiveStruct.test2.z() << std::endl;

	return 0;

}

