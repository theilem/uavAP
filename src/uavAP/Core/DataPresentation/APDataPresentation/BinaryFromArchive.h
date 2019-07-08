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
 * @file BinaryFromArchive.h
 * @brief Defines the BinaryFromArchive.
 * @date Aug 22, 2017
 * @author Mirco Theile, mirco.theile@tum.de
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_BINARYFROMARCHIVE_H_
#define UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_BINARYFROMARCHIVE_H_

#include <uavAP/Core/DataPresentation/APDataPresentation/ArchiveOptions.h>
#include <stdexcept>
#include <string>

/**
 * @brief Archive Error is a range error
 */
class ArchiveError: public std::range_error
{
public:
	ArchiveError(const std::string& what) :
			std::range_error(what)
	{
	}

};

/**
 * @brief Definition of a Binary from archive used to get data from a string.
 */
class BinaryFromArchive
{
public:

	/**
	 * @brief Contructor wrapping around a string
	 * @param str String to be read from
	 */
	BinaryFromArchive(const std::string& str, const ArchiveOptions& opts = ArchiveOptions());

	void
	setOptions(const ArchiveOptions& opts);

	/**
	 * @return return current first character of the string.
	 */
	const char*
	begin();

	/**
	 * @brief Move first character ptr back by bytes.
	 * @param bytes Bytes to be moved back on the string
	 */
	void
	consume(unsigned long bytes);

	void
	read(char* val, unsigned long bytes);

	/**
	 * @brief Flush out operator used for non protobuf objects
	 * @param val Non protobuf object
	 * @return From archive
	 */
	template<class Type>
	BinaryFromArchive&
	operator >>(Type& val);

	BinaryFromArchive&
	operator >>(double& doub);

	/**
	 * @brief Operator & defining behavior for flush out
	 * @param val Value to be flushed in or out.
	 */
	template<class Type>
	void
	operator &(Type& val);

	/**
	 * @brief Flush in operator. Does nothing since it is a "from archive"
	 * @param val Value to be flushed in.
	 * @return From archive
	 */
	template<class Type>
	BinaryFromArchive&
	operator <<(Type& val);

	/**
	 * @brief Returns the remaining string from idx_ to the end of string_
	 * @return remaining string
	 */
	std::string
	getRemaining();

	std::size_t
	getConsumed();
private:

	ArchiveOptions options_;

	const std::string& string_; //!< String to be parsed, but not modified

	unsigned int idx_; //!< Current first position of the parsing process

};

#endif /* UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_BINARYFROMARCHIVE_H_ */
