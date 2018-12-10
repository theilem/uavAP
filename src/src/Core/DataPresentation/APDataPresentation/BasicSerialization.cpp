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
 * PODSerialization.cpp
 *
 *  Created on: Aug 27, 2017
 *      Author: mircot
 */

#include "uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/BinaryFromArchive.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/BinaryToArchive.h"
#include <uavAP/Core/DataPresentation/APDataPresentation/FileFromArchive.h>
#include <uavAP/Core/DataPresentation/APDataPresentation/FileToArchive.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <cstring>

void
dp::load(BinaryFromArchive& ar, char* val, unsigned long bytes)
{
	memcpy(val, ar.begin(), bytes);
	ar.consume(bytes);
}

void
dp::store(BinaryToArchive& ar, char* val, unsigned long bytes)
{
	ar.append(val, bytes);
}

void
dp::load(FileFromArchive& ar, char* val, unsigned long bytes)
{
	ar.read(val, bytes);
}

void
dp::store(FileToArchive& ar, char* val, unsigned long bytes)
{
	ar.append(val, bytes);
}

void
dp::split(BinaryFromArchive& ar, char* val, std::size_t size)
{
	load(ar, val, size);
}

void
dp::split(BinaryToArchive& ar, char* val, std::size_t size)
{
	store(ar, val, size);
}

void
dp::split(FileFromArchive& ar, char* val, std::size_t size)
{
	load(ar, val, size);
}

void
dp::split(FileToArchive& ar, char* val, std::size_t size)
{
	store(ar, val, size);
}
