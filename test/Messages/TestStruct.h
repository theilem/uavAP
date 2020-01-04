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
 * TestStruct.h
 *
 *  Created on: Jun 30, 2017
 *      Author: mircot
 */

#ifndef TEST_MESSAGES_TESTSTRUCT_H_
#define TEST_MESSAGES_TESTSTRUCT_H_
#include "uavAP/Core/LinearAlgebra.h"

struct TestStruct
{
	int test1;
	double test3;
	Vector3 test2;
};

#endif /* TEST_MESSAGES_TESTSTRUCT_H_ */
