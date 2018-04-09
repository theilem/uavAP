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
 * @file APDataPresentation.h
 * @brief File defining the APDataPresentation and Macros for serialization and deseralization.
 * @date Aug 22, 2017
 * @author Mirco Theile, mircot@illinois.edu
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_APDATAPRESENTATION_H_
#define UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_APDATAPRESENTATION_H_

#include "uavAP/Core/DataPresentation/IDataPresentation.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/BinaryFromArchive.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/BinaryToArchive.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/detail/BinaryToArchiveImpl.hpp"
#include "uavAP/Core/DataPresentation/APDataPresentation/detail/BinaryFromArchiveImpl.hpp"
#include "uavAP/Core/DataPresentation/APDataPresentation/detail/BasicSerializationImpl.hpp"
#include "uavAP/Core/Object/IAggregatableObject.h"

#include <boost/property_tree/ptree.hpp>
#include "uavAP/Core/Logging/APLogger.h"

#include <memory>

#ifndef MAPPING
#error Mapping in dataPresentation is undefined. Include Content File before DataPresentation.
#endif

#define SERIALIZE(TYPE) 		archive << boost::any_cast<TYPE>(data); \
		break

#define DESERIALIZE(TYPE) 		TYPE data; \
		archive >> data; \
		return data

#define CONTENTMAPPING(ENUM,FUNC)	switch (ENUM){ 	\
		MAPPING(FUNC)			\
default: break;}

#define SERIALIZE_CONTENT(ENUM)		CONTENTMAPPING(ENUM,SERIALIZE)
#define DESERIALIZE_CONTENT(ENUM)	CONTENTMAPPING(ENUM,DESERIALIZE)


/**
 * @brief Defines the serialization of data represented by Content.
 */
template <typename Content, typename Target>
class APDataPresentation: public IDataPresentation<Content, Target>, public IAggregatableObject
{
public:

	/**
	 * @brief Default constructor
	 */
	APDataPresentation() = default;

	/**
	 * @brief Create a APDataPresentation shared_ptr.
	 * @param config configuration for the data presentation
	 * @return Shared ptr to a new object
	 */
	static std::shared_ptr<IDataPresentation<Content,Target>>
	create(const boost::property_tree::ptree& config)
	{
		auto dp = std::make_shared<APDataPresentation<Content,Target>>();
		dp->configure(config);
		return dp;
	}

	/**
	 * @brief Configuration of APDataPresentation. No config parameters needed.
	 * @param config Configuration. No config needed, yet.
	 * @return true on success
	 */
	bool
	configure(const boost::property_tree::ptree& config)
	{
		return true;
	}

	/**
	 * @brief Set the Target field inside a packet.
	 * @param packet Packet reference to be modified by adding the target
	 * @param target Target enum
	 */
	void
	setTarget(Packet& packet, const Target& target) override
	{
		std::string str;
		BinaryToArchive archive(str);
		archive << target;
		packet.prepend(str);
	}

	/**
	 * @brief Retrieves target from the packet.
	 * @param packet Packet containing target field.
	 * @return Target found in packet. Unknown behavior if no target is set in packet.
	 */
	Target
	getTarget(Packet& packet) override
	{
		BinaryFromArchive archive(packet.getBuffer());

		Target target;
		archive >> target;

		packet = Packet(archive.getRemaining());
		return target;
	}

	/**
	 * Interface implementation. Not needed.
	 */
	void
	notifyAggregationOnUpdate(Aggregator&) override
	{
	}

	/**
	 * @brief Serializes any data using the content to determine serialization method.
	 * @param data Data to be serialized. Any data type possible
	 * @param content Content defining serialization
	 * @return New packet containing the serialized data and the content
	 */
	Packet
	serialize(const boost::any& data, const Content& content)
	{
		std::string str;
		BinaryToArchive archive(str);
		try
		{
			archive << content;
			SERIALIZE_CONTENT(content)
		} catch (boost::bad_any_cast&)
		{
			APLOG_ERROR << "Content does not match data.";
			return Packet();
		}
		return Packet(str);
	}

	/**
	 * @brief Deserialize packet. Content is found in the packet and used for deserialzation.
	 * @param packet Packet to be deserialzied.
	 * @param content Content found in packet.
	 * @return Data object found in packet. Wrapped in boost any object. boost::any_cast needed.
	 */
	boost::any
	deserialize(const Packet& packet, Content& content)
	{
		if (packet.getSize() == 0)
		{
			content = Content::INVALID;
			return boost::any();
		}
		BinaryFromArchive archive(packet.getBuffer());
		try
		{
			uint8_t c;
			archive >> c;
			content = (Content)c;
			DESERIALIZE_CONTENT(content)
		} catch (ArchiveError& err)
		{
			APLOG_WARN << "Invalid Packet. Error: " << err.what();
			content = Content::INVALID;
			return boost::any();
		}

		return boost::any();
	}

private:
};

#endif /* UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_APDATAPRESENTATION_H_ */
