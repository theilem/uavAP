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
 * WindAnalysisStatus.h
 *
 *  Created on: Feb 8, 2019
 *      Author: mirco
 */

#ifndef UAVAP_MISSIONCONTROL_WINDANALYSIS_WINDANALYSISSTATUS_H_
#define UAVAP_MISSIONCONTROL_WINDANALYSIS_WINDANALYSISSTATUS_H_

#include <cpsCore/Utilities/LinearAlgebra.h>

struct WindInfo
{
	Vector3 velocity;
};

struct WindAnalysisStatus
{
	Vector3 velocity;
	FloatingType speed;
	FloatingType direction;
	bool manual;

	inline void
	reset()
	{
		velocity = {0, 0, 0};
		speed = 0;
		direction = 0;
		manual = false;
	}
};

namespace dp
{
template<class Archive, typename T>
inline void
serialize(Archive& ar, WindInfo& t)
{
	ar & t.velocity;
}

template<class Archive, typename T>
inline void
serialize(Archive& ar, WindAnalysisStatus& t)
{
	ar & t.velocity;
	ar & t.speed;
	ar & t.direction;
	ar & t.manual;
}
}

#endif /* UAVAP_MISSIONCONTROL_WINDANALYSIS_WINDANALYSISSTATUS_H_ */
