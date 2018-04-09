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
 * @file ISenderImpl.h
 * @brief Defines ISenderImpl
 * @date Jul 31, 2017
 * @author Mirco Theile, mircot@illinois.edu
 */
#ifndef UAVAP_CORE_IDC_ISENDERIMPL_H_
#define UAVAP_CORE_IDC_ISENDERIMPL_H_

class Packet;

/**
 * @brief Interface for the sender implementation. Allows to send packets through any IDC.
 */
class ISenderImpl
{

public:

	virtual
	~ISenderImpl() = default;

	/**
	 * @brief Send a packet through IDC that creates this SenderImpl.
	 * @param packet Packet to be send
	 * @return true on success
	 */
	virtual bool
	sendPacket(const Packet& packet) = 0;
};


#endif /* UAVAP_CORE_IDC_ISENDERIMPL_H_ */
