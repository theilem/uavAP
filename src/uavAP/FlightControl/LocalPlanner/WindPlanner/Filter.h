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
 * Filter.h
 *
 *  Created on: Jun 13, 2017
 *      Author: mircot
 */

#ifndef WINDPLANNER_FILTER_H_
#define WINDPLANNER_FILTER_H_

class Filter
{
public:
	Filter();
	Filter(float a0, float a1, float a2, float b0, float b1, float b2);
	void
	setConstants(float a0, float a1, float a2, float b0, float b1, float b2);
	float
	filter(float x);
private:
	float a0, a1, a2, b0, b1, b2;
	float xm1, xm2, ym1, ym2;
};

#endif /* WINDPLANNER_FILTER_H_ */
