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
 * @file LockTypes.h
 * @brief Standard lock typedefs
 * @date Jun 27, 2017
 * @author Mirco Theile, mirco.theile@tum.de
 */

#ifndef UAVAP_CORE_LOCKTYPES_H_
#define UAVAP_CORE_LOCKTYPES_H_

#ifdef NO_LOCKING

using Mutex = int;

struct FakeLock
{
	FakeLock(const Mutex& mut){}

	void
	unlock(){}
};
using Lock = FakeLock;
using LockGuard = int;

#else

#include <mutex>

using Mutex = std::mutex;
using Lock = std::unique_lock<Mutex>;
using LockGuard = std::lock_guard<Mutex>;
//using SharedLock = std::shared_lock<boost::shared_mutex>;
//using SharedLockGuard = boost::shared_lock_guard<boost::shared_mutex>;
//using ExclusiveLock = boost::unique_lock<boost::shared_mutex>;
//using ExclusiveLockGuard = boost::lock_guard<boost::shared_mutex>;
#endif

#endif /* UAVAP_CORE_LOCKTYPES_H_ */
