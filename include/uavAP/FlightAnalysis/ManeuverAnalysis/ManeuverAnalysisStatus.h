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
 * ManeuverAnalysisStatus.h
 *
 *  Created on: Aug 7, 2018
 *      Author: simonyu
 */

#ifndef UAVAP_FLIGHTANALYSIS_MANEUVERANALYSIS_MANEUVERANALYSISSTATUS_H_
#define UAVAP_FLIGHTANALYSIS_MANEUVERANALYSIS_MANEUVERANALYSISSTATUS_H_

#include <string>

struct ManeuverAnalysisStatus
{
	std::string maneuver;
	bool analysis;
	bool interrupted;

	ManeuverAnalysisStatus()
	{
		reset();
	}

	void reset()
	{
		maneuver = "";
		analysis = false;
		interrupted = false;
	}
};

namespace dp
{
template<class Archive, typename Type>
inline void
serialize(Archive& ar, ManeuverAnalysisStatus& t)
{
	ar & t.maneuver;
	ar & t.analysis;
	ar & t.interrupted;
}
} /* dp */

#endif /* UAVAP_FLIGHTANALYSIS_MANEUVERANALYSIS_MANEUVERANALYSISSTATUS_H_ */
