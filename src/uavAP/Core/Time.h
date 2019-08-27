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
 * @file Time.h
 * @brief Typedefs for usage of time in uavAP based on boost posix_time
 * @date Jun 25, 2017
 * @author Mirco Theile, mirco.theile@tum.de
 */

#ifndef UAVAP_CORE_TIME_H_
#define UAVAP_CORE_TIME_H_

#include "uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h"
#include <chrono>
#include <ctime>

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;
using Duration = Clock::duration;
using Nanoseconds = std::chrono::nanoseconds;
using Microseconds = std::chrono::microseconds;
using Milliseconds = std::chrono::milliseconds;
using Seconds = std::chrono::seconds;
using Minutes = std::chrono::minutes;
using Hours = std::chrono::hours;

inline std::tm*
getTM(const TimePoint& time)
{
	auto timeT = Clock::to_time_t(time);
	return std::localtime(&timeT);
}

inline Duration
getTimeOfDay(const TimePoint& time)
{
	Hours sinceEpoch = std::chrono::duration_cast<Hours>(time.time_since_epoch());
	return time.time_since_epoch() - Hours((sinceEpoch.count() / 24)*24);
}


namespace dp
{

template<class Archive, typename Type>
inline void
store(Archive& ar, const TimePoint& t)
{
	ar << t.time_since_epoch().count();
}

template<class Archive, typename Type>
inline void
load(Archive& ar, TimePoint& t)
{
	TimePoint::rep rep;
	ar >> rep;

	TimePoint tNew;
	tNew += TimePoint::duration(rep);
	t = tNew;
}

template<class Archive, typename Type>
inline void
serialize(Archive& ar, TimePoint& t)
{
	split(ar, t);
}
}

#endif /* UAVAP_CORE_TIME_H_ */
