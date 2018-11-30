/*
 * BinarySerialization.hpp
 *
 *  Created on: Jul 27, 2018
 *      Author: mircot
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_BINARYSERIALIZATION_HPP_
#define UAVAP_CORE_DATAPRESENTATION_BINARYSERIALIZATION_HPP_
#include "uavAP/Core/DataPresentation/APDataPresentation/BinaryFromArchive.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/BinaryToArchive.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/detail/BinaryFromArchiveImpl.hpp"
#include "uavAP/Core/DataPresentation/APDataPresentation/detail/BinaryToArchiveImpl.hpp"
#include "uavAP/Core/DataPresentation/APDataPresentation/detail/BasicSerializationImpl.hpp"
#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/Core/Logging/APLogger.h"

namespace dp
{

template<typename T>
inline Packet
serialize(const T& data)
{
	std::string str;
	BinaryToArchive archive(str);
	archive << data;
	return Packet(str);
}

template<typename T>
inline T
deserialize(const Packet& packet)
{
	if (packet.getSize() == 0)
	{
		APLOG_WARN << "packet empty";
		return T();
	}
	BinaryFromArchive archive(packet.getBuffer());
	try
	{
		T t;
		archive >> t;
		return t;
	} catch (ArchiveError& err)
	{
		APLOG_WARN << "Invalid Packet. Error: " << err.what();
		return T();
	}
}

template<typename T>
inline T
extract(Packet& packet)
{
	if (packet.getSize() == 0)
	{
		return T();
	}
	BinaryFromArchive archive(packet.getBuffer());
	try
	{
		T t;
		archive >> t;
		packet.consume(archive.getConsumed());
		return t;
	} catch (ArchiveError& err)
	{
		APLOG_WARN << "Invalid Packet. Error: " << err.what();
		return T();
	}
}

} //namespace dp

#endif /* UAVAP_CORE_DATAPRESENTATION_BINARYSERIALIZATION_HPP_ */
