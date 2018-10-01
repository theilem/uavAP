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
 * @file Packet.h
 * @brief Defines the packet class
 * @date Jul 17, 2017
 * @author Mirco Theile, mirco.theile@tum.de
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_PACKET_H_
#define UAVAP_CORE_DATAPRESENTATION_PACKET_H_

#include <string>

/**
 * @brief Describes the Packet class that contains a string representing data.
 */
class Packet
{

public:

	/**
	 * @brief Default constructor.
	 */
	Packet() = default;

	/**
	 * @brief Create a packet with a given string
	 * @param buf initial string of the packet
	 */
	Packet(const std::string& buf);

	/**
	 * @brief Add a string in front of the current buffer
	 * @param str String to be prepended
	 */
	void
	prepend(const std::string& str);

	void
	prepend(const Packet& str);

	/**
	 * @return Return a const reference to the buffer_
	 */
	const std::string&
	getBuffer() const;

	/**
	 * @return Return a const char pointer to the starting character of buffer_
	 */
	const char*
	getStart();

	/**
	 * @brief Get the size of the current buffer_.
	 * @return Size of the buffer
	 */
	std::size_t
	getSize() const;

	std::string
	consume(std::size_t length);

	uint16_t
	getCRC16() const;

private:

	std::string buffer_; //!< Buffer containing the information in serialized form
};


#endif /* UAVAP_CORE_DATAPRESENTATION_PACKET_H_ */
