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
 * IHeader.h
 *
 *  Created on: Jul 26, 2018
 *      Author: mircot
 */

#ifndef UAVAP_CORE_IDC_HEADER_H_
#define UAVAP_CORE_IDC_HEADER_H_
#include "uavAP/Core/DataPresentation/Packet.h"

struct Header
{
	virtual
	~Header() = default;

	void
	stripHeader(Packet& packet);

	void
	getHeader(const Packet& packet);

	void
	addHeader(Packet& packet);

	virtual std::size_t
	getLength() = 0;

	virtual std::string
	toString() = 0;

	virtual void
	fromString(const std::string& str) = 0;
};

#endif /* UAVAP_CORE_IDC_HEADER_H_ */
