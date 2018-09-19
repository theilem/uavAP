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
 * Filter.cpp
 *
 *  Created on: Jun 13, 2017
 *      Author: uav
 */
#include <FlightControl/LocalPlanner/WindPlanner/Filter.h"
#include <string.h"

Filter::Filter()
{
	memset(this, 0, sizeof(Filter));
}

Filter::Filter(float a0, float a1, float a2, float b0, float b1, float b2) :
		a0(a0), a1(a1), a2(a2), b0(b0), b1(b1), b2(b2), xm1(0), xm2(0), ym1(0), ym2(0)
{
}

void
Filter::setConstants(float a0, float a1, float a2, float b0, float b1, float b2)
{
	this->a0 = a0;
	this->a1 = a1;
	this->a2 = a2;
	this->b0 = b0;
	this->b1 = b1;
	this->b2 = b2;
}

float
Filter::filter(float x)
{
	float y = b0 * x + b1 * xm1 + b2 * xm2 - a1 * ym1 - a2 * ym2;
	xm2 = xm1;
	xm1 = x;
	ym2 = ym1;
	ym1 = y;
	return y;
}
