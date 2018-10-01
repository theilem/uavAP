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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian_calendar.hpp>
#include "uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h"

using TimePoint = boost::posix_time::ptime;
using Duration = boost::posix_time::time_duration;
using Microseconds = boost::posix_time::microseconds;
using Milliseconds = boost::posix_time::milliseconds;
using Seconds = boost::posix_time::seconds;
using Minutes = boost::posix_time::minutes;
using Hours = boost::posix_time::hours;
using Date = boost::gregorian::date;

namespace dp
{
template<class Archive, typename Type>
inline void
store(Archive& ar, const TimePoint& t)
{
	auto special = t.is_special();
	ar << special;
	if (!special)
	{
		ar << t.date().day_count().as_number();
		ar << t.time_of_day().total_microseconds();
	}
}

template<class Archive, typename Type>
inline void
load(Archive& ar, TimePoint& t)
{
	bool special;
	ar >> special;
	if (special)
	{
		t = boost::posix_time::not_a_date_time;
		return;
	}
	Date::date_rep_type::int_type days;
	ar >> days;
	TimePoint::time_duration_type::tick_type micros;
	ar >> micros;

	t = TimePoint(Date(days), Microseconds(micros));
}

template<class Archive, typename Type>
inline void
serialize(Archive& ar, TimePoint& t)
{
	split(ar, t);
}
}

#endif /* UAVAP_CORE_TIME_H_ */
