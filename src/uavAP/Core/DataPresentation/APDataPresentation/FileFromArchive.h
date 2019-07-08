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
 * @file FileFromArchive.h
 * @date Nov 12, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_FILEFROMARCHIVE_H_
#define UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_FILEFROMARCHIVE_H_
#include "uavAP/Core/DataPresentation/APDataPresentation/ArchiveOptions.h"
#include <iostream>
#include <type_traits>


class FileFromArchive
{

public:

	FileFromArchive(std::ifstream& file, const ArchiveOptions& opts = ArchiveOptions());

	void
	setOptions(const ArchiveOptions& opts);

	void
	read(char* val, unsigned long bytes);

	template<class Type>
	FileFromArchive&
	operator >>(Type& val);

	FileFromArchive&
	operator >>(double& doub);

	template<class Type>
	void
	operator &(Type& val);

	template<class Type>
	FileFromArchive&
	operator <<(Type& val);

private:

	ArchiveOptions options_;

	std::ifstream& file_;
};

#endif /* UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_FILEFROMARCHIVE_H_ */
