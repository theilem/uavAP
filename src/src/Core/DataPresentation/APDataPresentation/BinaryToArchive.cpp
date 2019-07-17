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
 * BinaryOutArchive.cpp
 *
 *  Created on: Aug 24, 2017
 *      Author: mircot
 */
#include "uavAP/Core/DataPresentation/APDataPresentation/BinaryToArchive.h"
#include <uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h>

BinaryToArchive::BinaryToArchive(std::string& str, const ArchiveOptions& opts) :
options_(opts), string_(str)
{
}

void
BinaryToArchive::append(const char* c, size_t length)
{
	string_.append(c, length);
}

void
BinaryToArchive::setOptions(const ArchiveOptions& opts)
{
	options_ = opts;
}

BinaryToArchive&
BinaryToArchive::operator <<(const double& doub)
{
	if (options_.compressDouble())
	{
		float flo = static_cast<float>(doub);
		dp::store(*this, reinterpret_cast<char*>(&flo), sizeof(float));
	}
	else
	{
		dp::store(*this, reinterpret_cast<char*>(&const_cast<double&>(doub)), sizeof(double));
	}
	return *this;
}
