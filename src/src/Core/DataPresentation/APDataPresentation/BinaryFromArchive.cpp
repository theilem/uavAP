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
 * BinaryInArchive.cpp
 *
 *  Created on: Aug 24, 2017
 *      Author: mircot
 */
#include "uavAP/Core/DataPresentation/APDataPresentation/BinaryFromArchive.h"

BinaryFromArchive::BinaryFromArchive(const std::string& str) :
		string_(str),
		idx_(0)
{
	static_assert(sizeof(double) == 8, "Double precision is not 8Byte");
}

const char*
BinaryFromArchive::begin()
{
	if (idx_ >= string_.size())
		throw ArchiveError("Archive idx too large.");
	return &string_[idx_];
}

void
BinaryFromArchive::consume(unsigned long bytes)
{
	idx_ += bytes;
}

std::string
BinaryFromArchive::getRemaining()
{
	return string_.substr(idx_, string_.size() - 1);
}
