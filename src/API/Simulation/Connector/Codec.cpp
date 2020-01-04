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
 * Codec.cpp
 *
 *  Created on: Jul 28, 2017
 *      Author: mircot
 */

#include "uavAP/API/Simulation/Connector/Codec.h"
#include <cstddef>

std::string
decode(std::string encoded)
{
	std::string result;
	for (unsigned int k = 0; k < encoded.size(); k++)
	{
		char val = encoded[k];
		char val1 = ((val & 0xF0) >> 4) + 0x2A;
		char val2 = (val & 0x0F) + 0x2A;

		if (val1 == 0x2A)
			break;
		result += val1;
		if (val2 == 0x2A)
			break;
		result += val2;
	}
	return result;
}

std::string
encode(std::string source)
{
	std::string result;
	bool toggle = false;
	for (unsigned int k = 0; k < source.size(); k++)
	{
		char val = source[k];
		val -= 0x2A;

		if (toggle)
			result[result.size() - 1] += (val & 0x0F) << 4;
		else
			result += val & 0x0F;
		toggle = !toggle;
	}
	return result;
}
