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
 * @file FileToArchiveImpl.hpp
 * @date Nov 13, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_DETAIL_FILETOARCHIVEIMPL_HPP_
#define UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_DETAIL_FILETOARCHIVEIMPL_HPP_
#include "uavAP/Core/DataPresentation/APDataPresentation/FileToArchive.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h"

template<typename Type>
FileToArchive&
FileToArchive::operator <<(const Type& cval)
{
	dp::serialize<FileToArchive, Type>(*this, const_cast<Type&>(cval));
	return *this;
}

template<class Type>
inline void
FileToArchive::operator &(const Type& val)
{
	*this << val;
}

template<class Type>
inline FileToArchive&
FileToArchive::operator >>(const Type& val)
{
	return *this;
}

#endif /* UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_DETAIL_FILETOARCHIVEIMPL_HPP_ */
