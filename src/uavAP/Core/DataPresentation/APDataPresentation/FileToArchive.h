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
 * @file FileToArchive.h
 * @date Nov 13, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_FILETOARCHIVE_H_
#define UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_FILETOARCHIVE_H_
#include <uavAP/Core/DataPresentation/APDataPresentation/ArchiveOptions.h>
#include <string>

class FileToArchive
{
public:
	/**
	 * @brief Constructor wrapping around a string
	 * @param str String to be wrapped around and filled with serialization
	 */
	FileToArchive(std::ofstream& str, const ArchiveOptions& opts = ArchiveOptions());

	void
	setOptions(const ArchiveOptions& opts);

	/**
	 * @brief Append the string with length characters from c
	 * @param c Characters to be appended
	 * @param length Number of characters to be appended
	 */
	void
	append(const char* c, size_t length);

	template<typename Type>
	FileToArchive&
	operator <<(const Type& cval);

	/**
	 * @brief Handle double according to setting compressDouble_
	 * @param doub
	 * @return
	 */
	FileToArchive&
	operator <<(const double& doub);

	/**
	 * @brief Operator & for flush in
	 * @param val Data to be flushed in
	 */
	template<class Type>
	void
	operator &(const Type& val);

	/**
	 * @brief Flush out operator. Does nothing
	 * @param val Data to be flushed out.
	 * @return The archive itself
	 */
	template<class Type>
	FileToArchive&
	operator >>(const Type& val);

private:

	ArchiveOptions options_;

	std::ofstream& file_;
};

#endif /* UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_FILETOARCHIVE_H_ */
