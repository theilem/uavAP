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
 * @file FileToArchive.cpp
 * @date Nov 13, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief
 */
#include <uavAP/Core/DataPresentation/APDataPresentation/FileToArchive.h>
#include <uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h>

#include <sstream>
#include <iostream>
#include <fstream>



FileToArchive::FileToArchive(std::ofstream& file, const ArchiveOptions& opts):
options_(opts), file_(file)
{
}

void
FileToArchive::setOptions(const ArchiveOptions& opts)
{
	options_ = opts;
}

void
FileToArchive::append(const char* c, size_t length)
{
	file_.write(c, length);
}

FileToArchive&
FileToArchive::operator <<(const double& doub)
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
