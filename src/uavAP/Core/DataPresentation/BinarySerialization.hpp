/*
 * BinarySerialization.hpp
 *
 *  Created on: Jul 27, 2018
 *      Author: mircot
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_BINARYSERIALIZATION_HPP_
#define UAVAP_CORE_DATAPRESENTATION_BINARYSERIALIZATION_HPP_
#include "uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/BinaryFromArchive.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/BinaryToArchive.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/FileFromArchive.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/FileToArchive.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/detail/BinaryFromArchiveImpl.hpp"
#include "uavAP/Core/DataPresentation/APDataPresentation/detail/BinaryToArchiveImpl.hpp"
#include "uavAP/Core/DataPresentation/APDataPresentation/detail/FileFromArchiveImpl.hpp"
#include "uavAP/Core/DataPresentation/APDataPresentation/detail/FileToArchiveImpl.hpp"
#include "uavAP/Core/DataPresentation/APDataPresentation/detail/BasicSerializationImpl.hpp"
#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/Core/Logging/APLogger.h"

namespace dp
{

template<typename T>
inline Packet
serialize(const T& data, const ArchiveOptions& options = ArchiveOptions())
{
	std::string str;
	BinaryToArchive archive(str, options);
	archive << data;
	return Packet(str);
}

template<typename T>
inline void
serialize(const T& data, std::ofstream& file, const ArchiveOptions& options = ArchiveOptions())
{
	FileToArchive archive(file, options);
	archive << data;
}

template<typename T>
inline T
deserialize(const Packet& packet, const ArchiveOptions& options = ArchiveOptions())
{
	if (packet.getSize() == 0)
	{
		APLOG_WARN << "packet empty";
		return T();
	}
	BinaryFromArchive archive(packet.getBuffer(), options);
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
deserialize(std::ifstream& file, const ArchiveOptions& options = ArchiveOptions())
{
	FileFromArchive archive(file, options);
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
extract(Packet& packet, const ArchiveOptions& options = ArchiveOptions())
{
	if (packet.getSize() == 0)
	{
		return T();
	}
	BinaryFromArchive archive(packet.getBuffer(), options);
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
