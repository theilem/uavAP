/**
 * @file FileToArchiveImpl.hpp
 * @date Nov 13, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_DETAIL_FILETOARCHIVEIMPL_HPP_
#define UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_DETAIL_FILETOARCHIVEIMPL_HPP_
#include <uavAP/Core/DataPresentation/APDataPresentation/FileToArchive.h>
#include "uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h"




template<typename Type>
inline typename std::enable_if<!std::is_base_of<google::protobuf::Message, Type>::value,
		FileToArchive>::type&
FileToArchive::operator <<(const Type& cval)
{
	dp::serialize<FileToArchive, Type>(*this, const_cast<Type&>(cval));
	return *this;
}

template<class Type>
inline typename std::enable_if<std::is_base_of<google::protobuf::Message, Type>::value,
		FileToArchive>::type&
FileToArchive::operator <<(const Type& message)
{
	std::string s;
	*this << static_cast<uint16_t>(s.size());
	message.SerializeToOstream(&file_);
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
