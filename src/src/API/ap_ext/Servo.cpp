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
 * ServoMapping.cpp
 *
 *  Created on: Sep 2, 2017
 *      Author: mircot
 */
#include "uavAP/API/ap_ext/Servo.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"

Servo::Servo() :
		servoMin_(4000),
		servoCenter_(6000),
		servoMax_(8000)
{
}

bool
Servo::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	pm.add("min", servoMin_, true);
	pm.add("max", servoMax_, true);
	if (!pm.add("center", servoCenter_, false))
	{
		servoCenter_ = (servoMin_ + servoMax_) / 2;
	}

	return pm.map();
}

unsigned long
Servo::map(double val)
{
	if (val < 0)
		return servoCenter_ + val * (servoCenter_ - servoMin_);
	return val * (servoMax_ - servoCenter_) + servoCenter_;
}

double
Servo::unmap(unsigned long val)
{
	double d = val;
	if (d < servoCenter_)
		return (d - servoCenter_) / (servoCenter_ - servoMin_);
	return (d - servoCenter_) / (servoMax_ - servoCenter_);
}
