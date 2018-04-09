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
 * TestWatchdog.cpp
 *
 *  Created on: Jun 30, 2017
 *      Author: mircot
 */

#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/process.hpp>
#include <iostream>

int
main(int argc, char** argv)
{
	boost::interprocess::message_queue::remove("test_message");

	boost::interprocess::message_queue message(boost::interprocess::create_only, "test_message", 10,
			1000);

	boost::process::child sender("./TestSender");
	boost::process::child receiver("./TestReceiver");

	sender.wait();

	std::cout << "Sender finished" << std::endl;
	receiver.wait();

	std::cout << "Receiver finished" << std::endl;

	message.remove("test_message");

}
