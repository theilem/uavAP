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
 * @file Sender.h
 * @brief Defines Sender
 * @date Jul 31, 2017
 * @author Mirco Theile, mircot@illinois.edu
 */

#ifndef UAVAP_CORE_IDC_SENDER_H_
#define UAVAP_CORE_IDC_SENDER_H_
#include <memory>

class ISenderImpl;
class Packet;

/**
 * @brief Generic Sender that is used for all IDC. The senderImpl_ takes care of the actual sending. Allows
 * 		to send packets.
 */
class Sender
{
public:

	/**
	 * @brief Default constructor
	 */
	Sender() = default;

	/**
	 * @brief Constructor setting the sender implementation
	 * @param impl Sender implementation that takes care of the IDC specific sending.
	 */
	Sender(std::shared_ptr<ISenderImpl> impl);

	/**
	 * @brief Send a packet using the sender implementation.
	 * @param packet Packet to be send
	 * @return true on success
	 */
	bool
	sendPacket(const Packet& packet);

	/**
	 * @return true if senderImpl_ is set and functional
	 */
	bool
	isConnected();

private:

	std::weak_ptr<ISenderImpl> senderImpl_; //!< Sender implementation that is set by IDC
};



#endif /* UAVAP_CORE_IDC_SENDER_H_ */
