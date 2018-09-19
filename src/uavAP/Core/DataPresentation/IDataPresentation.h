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
 * @file IDataPresentation.h
 * @brief Definition of the Data presentation interface
 * @date 28 July 2017
 * @author Mirco Theile, mircot@illinois.edu
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_IDATAPRESENTATION_H_
#define UAVAP_CORE_DATAPRESENTATION_IDATAPRESENTATION_H_

#include <boost/any.hpp>
#include <unordered_map>

#include "uavAP/Core/DataPresentation/Packet.h"

/**
 * @brief Data presentation interface.
 *
 * Template defines the enum for Content and Target.
 */
template<typename Content, typename Target>
class IDataPresentation
{
public:

	/**
	 * @brief virtual default destructor
	 */
	virtual
	~IDataPresentation() = default;

	/**
	 * @brief Interface for serialization of any data. Creates a packet based on the Content.
	 * @param data Data to be serialized. Can be any data type.
	 * @param content Content enum representing the data type.
	 * @return Packet with the serialized string of data.
	 */
	virtual Packet
	serialize(const boost::any& data, const Content& content) = 0;

	/**
	 * @brief Interface for deserialization of packet. Figures out the content by looking at the packet.
	 *
	 * boost::any_cast has to be performed to get the actual data object. Cast parameter should be selected
	 * according to the content found in the packet and returned through the content reference.
	 * @param packet Containing the Content information and the data
	 * @param content Will contain the content found in packet.
	 * @return Returns the found data wrapped in a boost any.
	 */
	virtual boost::any
	deserialize(const Packet& packet, Content& content) = 0;

	/**
	 * @brief Set the target for a given packet.
	 * @param packet Packet that should be extended with target information
	 * @param target Target information
	 */
	virtual void
	setTarget(Packet& packet, const Target& target) = 0;

	/**
	 * @brief Get the target information contained in packet.
	 * @param packet Packet with Target information
	 * @return Target information
	 */
	virtual Target
	getTarget(Packet& packet) = 0;
};

#endif /* UAVAP_CORE_DATAPRESENTATION_IDATAPRESENTATION_H_ */
