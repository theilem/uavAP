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
/**
 * @file MessageObject.h
 * @author Mirco Theile, mirco.theile@tum.de
 * @date [DD.MM.YYYY] 18.07.2017
 */

#ifndef UAVAP_CORE_IPC_MESSAGEOBJECT_H_
#define UAVAP_CORE_IPC_MESSAGEOBJECT_H_
#include <boost/interprocess/sync/interprocess_condition_any.hpp>
#include <boost/interprocess/sync/interprocess_sharable_mutex.hpp>
#include <atomic>

struct MessageObjectHeader
{
	boost::interprocess::interprocess_condition_any cnd;
	boost::interprocess::interprocess_sharable_mutex mtx;

	bool active;

	std::size_t maxPacketSize;
	std::size_t packetSize;

	MessageObjectHeader() :
			active(true), maxPacketSize(0), packetSize(0)
	{
	}
};

template<class Object>
struct MessageObject: public Object
{
	MessageObjectHeader header;

	MessageObject() :
			Object()
	{
	}

};

#endif /* UAVAP_CORE_IPC_MESSAGEOBJECT_H_ */
